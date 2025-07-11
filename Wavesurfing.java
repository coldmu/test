package contest;

import robocode.*;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.*;

/**
 * Wavesurfing - Wave Surfing 전략 기반 Robocode 봇
 *
 * <리팩토링 및 한글 주석 강화 버전>
 *
 * - 적의 파동(Wave) 추적 및 위험 예측, 회피 전략 적용
 * - Markov Chain 기반 적 이동 예측, GuessFactor, 벽 스무딩, 중력장 이동 등 다양한 기법 사용
 */
public class Wavesurfing extends AdvancedRobot {
    // =====================
    // === 주요 변수 및 상수 ===
    // =====================
    private Enemy enemy = new Enemy(); // 적 정보 저장용 객체
    private ArrayList<Wave> waves = new ArrayList<>(); // 적이 쏜 파동(총알) 정보 저장
    private static int[] surfStats = new int[31]; // GuessFactor 기반 위험도 통계 [-15~+15]
    private static final double MAX_SURF_DISTANCE = 400; // Wave Surfing 발동 거리(px)

    // 이동/레이더 관련 변수
    private double moveDirection = 1; // 이동 방향 (1: 정방향, -1: 역방향)

    // 필드 및 상수
    private static final double GUN_WALL_PADDING = 17.5; // 총알 피격시 벽 여유
    private static final int NAV_WALL_PADDING = 25;      // 내비게이션 벽 여유
    private static final int AIM_RESOLUTION = 1000;      // 타겟팅 분할 해상도
    private static final int STATE_TABLE_SIZE = 126;     // Markov 테이블 크기
    private static final int ENEMY_HASH_SIZE = 256;      // 적 구분 해시 크기

    private static double fieldWidth = 0;  // 전장 가로
    private static double fieldHeight = 0; // 전장 세로

    // Markov 예측 테이블 [적][상태][테이블]
    private static int[][][] transitionMatrix = new int[ENEMY_HASH_SIZE][579][STATE_TABLE_SIZE + 1];

    // 적 정보
    private static String enemyId = null;                // 적 이름
    private static double enemyDistance = Double.POSITIVE_INFINITY; // 적과의 거리
    private static double enemySpeed = 0;                // 적 속도
    private static double enemyHeading = 0;              // 적 헤딩
    private static int enemyMarkovState = 0;             // 적 상태 (Markov)

    // 내 위치/상태
    private static double selfX = 0;
    private static double selfY = 0;
    private static int travelDirection = 1;              // 내 이동 방향

    private static boolean meleeMode = true; // 근접전/난전 모드 여부

    // =======================
    // === 메인 루프 및 초기화 ===
    // =======================
    /**
     * 봇의 메인 루프 - 초기화 및 레이더/총 설정
     */
    public void run() {
        setAdjustGunForRobotTurn(true); // 몸체 회전 시 총 방향 고정
        setAdjustRadarForGunTurn(true); // 총 회전 시 레이더 고정
        setAdjustRadarForRobotTurn(true); // 몸체 회전 시 레이더 고정

        fieldWidth = getBattleFieldWidth();
        fieldHeight = getBattleFieldHeight();
        turnRadarRightRadians(Double.POSITIVE_INFINITY); // 무한 레이더 회전
        setTurnRadarRightRadians(enemyDistance = Double.POSITIVE_INFINITY); // 레이더 초기화
        execute();
    }


    // ================================
    // === 적 스캔 및 주요 이벤트 처리 ===
    // ================================
    /**
     * 적을 스캔했을 때 호출되는 이벤트 핸들러
     * - 적 발포 감지 및 Wave 생성
     * - Wave Surfing/회피, 타겟팅, Markov 예측 등 수행
     */
    public void onScannedRobot(ScannedRobotEvent e) {
        // 남은 적 수에 따라 근접전/난전 모드 결정
        meleeMode = getOthers() >= 3; // 3명 이상 남으면 난전 모드
        System.out.println("meleeMode: " + meleeMode);

        // 적이 발사한 것으로 추정될 때 Wave(총알) 정보 생성
        // 적이 총을 쏜 것으로 추정되면 Wave(총알) 정보 생성
        if (enemy.energy > 0 && enemy.energy > e.getEnergy() &&
                enemy.energy - e.getEnergy() <= 3.0 && enemy.energy - e.getEnergy() >= 0.1) {
            double bulletPower = enemy.energy - e.getEnergy();
            Wave wave = new Wave();
            wave.sourceX = enemy.x;
            wave.sourceY = enemy.y;
            wave.startBearing = enemy.bearing;
            wave.startTime = getTime() - 1;
            wave.bulletVelocity = 20 - 3 * bulletPower;
            waves.add(wave);
            System.out.println("wave: " + wave);
        }
        // 적 정보 갱신
        enemy.update(e, this);
        // 지나간 Wave(총알) 제거
        updateWaves();

        // Wave가 가까우면 Wave Surfing(회피), 아니면 기본 이동
        Wave closestWave = getClosestWave();
        if (closestWave != null) {
            double distanceToWave = Point2D.distance(closestWave.sourceX, closestWave.sourceY, getX(), getY())
                    - closestWave.distanceTraveled(getTime());
            if (distanceToWave <= MAX_SURF_DISTANCE && !meleeMode) {
                System.out.println("doSurfing");
                doSurfing();
            }
        }

        // === Markov 기반 타겟팅/예측 ===
        String scannedName = e.getName();
        // 새로운 적이면 Markov 테이블 인덱스 초기화
        if (enemyId == null || !scannedName.equals(enemyId)) {
            int enemyHash = Math.abs(scannedName.hashCode()) % ENEMY_HASH_SIZE;
            int[][] table = transitionMatrix[enemyHash];
            for (int[] t : table) {
                t[STATE_TABLE_SIZE] = 0;
            }
        }
        double enemySpeedNow = e.getVelocity();
        double scannedDist = e.getDistance();
        double scannedAbsBearing = getHeadingRadians() + e.getBearingRadians();

        // 적과의 거리가 가까워지거나 같은 적일 때 타겟팅/예측 수행
        if (scannedDist < enemyDistance || scannedName.equals(enemyId)) {
            enemyId = scannedName;
            lockRadar(scannedAbsBearing); // 레이더 고정
            double bulletPower = calculateBulletPower(getEnergy(), scannedDist); // 발사 에너지 계산
            fireIfReady(bulletPower, scannedDist); // 발사
            int state = calculateMarkovState(enemySpeedNow, enemySpeed, enemyHeading, e.getHeadingRadians()); // 상태 계산
            int bestBin = predictAndKernelDensity(state, scannedAbsBearing, scannedDist, enemySpeedNow, bulletPower); // 예측
            aimGun(bestBin); // 조준
            updateMarkovTable(state); // 테이블 갱신
            updateEnemyInfo(state, enemySpeedNow, e.getHeadingRadians()); // 적 정보 갱신
        }
        execute(); // 명령 실행
    }


    private void updateWaves() {
        for (int i = waves.size() - 1; i >= 0; i--) {
            Wave wave = waves.get(i);
            if (wave.hasPassedMe(getX(), getY(), getTime())) {
                waves.remove(i);
            }
        }
    }

    private void doSurfing() {
        Wave surfWave = getClosestWave();
        if (surfWave == null)
            return;

        double dangerLeft = checkDanger(surfWave, -1);
        double dangerRight = checkDanger(surfWave, 1);

        double goAngle = absoluteBearing(surfWave.sourceX, surfWave.sourceY, getX(), getY());

        if (dangerLeft < dangerRight) {
            goAngle = goAngle - Math.PI / 2;
        } else {
            goAngle = goAngle + Math.PI / 2;
        }

        // 벽 스무딩 적용
        goAngle = wallSmoothing(getX(), getY(), goAngle, dangerLeft < dangerRight ? -1 : 1);
        
        // anti-gravity adjustment: 벽과 중앙이 아닌 중력 벡터를 결합
        double gravAngle = calcGravityAngle();
        double sin = 0.7 * Math.sin(goAngle) + 0.3 * Math.sin(gravAngle);
        double cos = 0.7 * Math.cos(goAngle) + 0.3 * Math.cos(gravAngle);
        goAngle = Math.atan2(sin, cos);
        
        // 이동 실행
        setBackAsFront(this, goAngle);
    }

    // =============================
    // === 위험도 예측 및 보조 메서드 ===
    // =============================
    /**
     * 해당 방향으로 이동 시 위험도를 계산 (GuessFactor 기반)
     */
    private double checkDanger(Wave wave, int direction) {
        Point2D.Double predictedPos = predictPosition(wave, direction);
        if (predictedPos != null) {
            int index = getFactorIndex(wave, predictedPos);
            if (index >= 0 && index < surfStats.length) {
                return surfStats[index];
            }
        }
        return 0;
    }

    /**
     * 해당 방향으로 이동을 예측하여 최종 위치 반환
     */
    private Point2D.Double predictPosition(Wave wave, int direction) {
        Point2D.Double predictedPosition = new Point2D.Double(getX(), getY());
        double predictedVelocity = getVelocity();
        double predictedHeading = getHeadingRadians();

        int counter = 0;
        boolean intercepted = false;

        do {
            double moveAngle = absoluteBearing(wave.sourceX, wave.sourceY,
                    predictedPosition.x, predictedPosition.y) + (direction * (Math.PI / 2));

            moveAngle = normalizeRelativeAngle(moveAngle - predictedHeading);

            double moveDir = 1;
            if (Math.cos(moveAngle) < 0) {
                moveAngle += Math.PI;
                moveDir = -1;
            }

            moveAngle = normalizeRelativeAngle(moveAngle);

            double maxTurning = Math.PI / 720d * (40d - 3d * Math.abs(predictedVelocity));
            predictedHeading = normalizeRelativeAngle(predictedHeading +
                    limit(-maxTurning, moveAngle, maxTurning));

            predictedVelocity += (predictedVelocity * moveDir < 0 ? 2 * moveDir : moveDir);
            predictedVelocity = limit(-8, predictedVelocity, 8);

            predictedPosition.x += Math.sin(predictedHeading) * predictedVelocity;
            predictedPosition.y += Math.cos(predictedHeading) * predictedVelocity;

            counter++;

            if (wave.hasPassedPosition(predictedPosition.x, predictedPosition.y,
                    (long) (wave.startTime + counter))) {
                intercepted = true;
            }
        } while (!intercepted && counter < 50); // 최대 50틱 예측

        return predictedPosition;
    }

    /**
     * 벽 스무딩: 이동 각도를 조정하여 벽에 너무 가까이 가지 않도록 보정
     */
    private double wallSmoothing(double botX, double botY, double angle, int orientation) {
        double smoothedAngle = angle;
        final double wallStick = 120;      // 테스트용 투사 거리
        final int maxIterations = 30;
        int tries = 0;

        while (!fieldRectangle(18).contains(projectMotion(botX, botY, smoothedAngle, wallStick))
                && tries < maxIterations) {
            smoothedAngle += orientation * 0.05; // 미세 조정
            tries++;
        }
        return smoothedAngle;
    }

    private Point2D.Double projectMotion(double x, double y, double angle, double distance) {
        return new Point2D.Double(x + Math.sin(angle) * distance, y + Math.cos(angle) * distance);
    }

    private Rectangle2D.Double fieldRectangle(double margin) {
        return new Rectangle2D.Double(margin, margin,
                getBattleFieldWidth() - margin * 2, getBattleFieldHeight() - margin * 2);
    }

    /**
     * 내 위치에서 가장 가까운 Wave(총알) 반환
     */
    private Wave getClosestWave() {
        double closestDistance = Double.POSITIVE_INFINITY;
        Wave closestWave = null;

        for (Wave wave : waves) {
            double distance = Point2D.distance(wave.sourceX, wave.sourceY, getX(), getY()) -
                    wave.distanceTraveled(getTime());
            if (distance > 0 && distance < closestDistance) {
                closestDistance = distance;
                closestWave = wave;
            }
        }
        return closestWave;
    }

    // =====================
    // === 이벤트 핸들러 ===
    // =====================
    /**
     * 총알에 맞았을 때 호출되는 이벤트 - 회피 및 위험 통계 갱신
     */
    public void onHitByBullet(HitByBulletEvent e) {
        moveDirection *= -1; // 이동 방향 반전
        setTurnRightRadians(moveDirection * Math.PI / 4);
        setAhead(100);

        // Wave 찾아서 위험도 증가
        Wave hitWave = findHitWave();
        if (hitWave != null) {
            int index = getFactorIndex(hitWave, new Point2D.Double(getX(), getY()));
            if (index >= 0 && index < surfStats.length) {
                surfStats[index] += 10;
            }
        }
    }

    /**
     * 벽에 부딪혔을 때 호출되는 이벤트 - 이동 방향 반전
     */
    public void onHitWall(HitWallEvent e) {
        moveDirection *= -1;
        setBack(50);
        setTurnRightRadians(moveDirection * Math.PI / 2);
    }

    /**
     * 내 위치에서 맞은 Wave(총알)를 찾음
     */
    private Wave findHitWave() {
        for (Wave wave : waves) {
            if (Math.abs(wave.startTime - (getTime() -
                    Point2D.distance(wave.sourceX, wave.sourceY, getX(), getY()) / wave.bulletVelocity)) < 3) {
                return wave;
            }
        }
        return null;
    }

    /**
     * GuessFactor 인덱스 계산 (위험도 통계용)
     */
    private int getFactorIndex(Wave wave, Point2D.Double targetLocation) {
        double offsetAngle = normalizeRelativeAngle(
                absoluteBearing(wave.sourceX, wave.sourceY, targetLocation.x, targetLocation.y) - wave.startBearing);
        double maxAngle = maxEscapeAngle(wave.bulletVelocity);

        if (maxAngle == 0)
            return 15; // 중앙값 반환

        double factor = offsetAngle / maxAngle;
        return (int) limit(0, (factor * 15) + 15, 30);
    }

    /**
     * 최대 회피 각도 계산
     */
    private double maxEscapeAngle(double velocity) {
        return Math.asin(Math.min(1.0, 8.0 / velocity));
    }

    // ======================
    // === 유틸리티 메서드 ===
    // ======================
    private double absoluteBearing(double x1, double y1, double x2, double y2) {
        return Math.atan2(x2 - x1, y2 - y1);
    }

    private double limit(double min, double value, double max) {
        return Math.max(min, Math.min(value, max));
    }

    private double normalizeRelativeAngle(double angle) {
        while (angle > Math.PI) angle -= 2.0 * Math.PI;
        while (angle < -Math.PI) angle += 2.0 * Math.PI;
        return angle;
    }

    /**
     * 벽과의 거리 유지를 위한 중력장 각도 계산
     */
    private double calcGravityAngle() {
        double x = getX(), y = getY();
        double w = getBattleFieldWidth(), h = getBattleFieldHeight();
        final double forceConst = 5000; // 벽 반발력 상수
        // X, Y축 벽 반발력 계산
        double fx = forceConst/(x*x) - forceConst/((w - x)*(w - x));
        double fy = forceConst/(y*y) - forceConst/((h - y)*(h - y));
        return Math.atan2(fx, fy);
    }

    private void setBackAsFront(AdvancedRobot robot, double goAngle) {
        // 동적 거리 유지: 적으로부터 너무 멀어지지 않도록 enemy.distance 기반
        double travelDistance = Math.max(50, Math.min(100, enemy.distance * 0.8));
        double angle = normalizeRelativeAngle(goAngle - robot.getHeadingRadians());

        if (Math.abs(angle) > (Math.PI / 2)) {
            if (angle < 0) {
                robot.setTurnRightRadians(Math.PI + angle);
            } else {
                robot.setTurnLeftRadians(Math.PI - angle);
            }
            robot.setBack(travelDistance);
        } else {
            if (angle < 0) {
                robot.setTurnLeftRadians(-angle);
            } else {
                robot.setTurnRightRadians(angle);
            }
            robot.setAhead(travelDistance);
        }
    }

    /**
     * 상태 이벤트(매 틱마다 호출) - 근접전 코너 내비게이션 (MeleeSeed 참고)
     */
    public void onStatus(StatusEvent e) {
        int navX = NAV_WALL_PADDING + 30 + (int)(enemyDistance / 2.5);
        int navY = NAV_WALL_PADDING;

        // 목적지 도달 시 방향 반전
        if (getDistanceRemaining() == 0) {
            travelDirection = -travelDirection;
        }

        // 이동 방향에 따라 목적지 좌표 설정
        if (travelDirection > 0) {
            navY = navX;
            navX = NAV_WALL_PADDING;
        }

        // 필드 중심 기준 반대편 코너로 이동
        selfX = getX();
        selfY = getY();
        if (selfX > fieldWidth / 2) {
            navX = (int)fieldWidth - navX;
        }
        if (selfY > fieldHeight / 2) {
            navY = (int)fieldHeight - navY;
        }

        // 목적지까지의 회전 각도 계산
        double turnAngle = absoluteBearing(navX, navY) - getHeadingRadians();

        // 빠른 방향 전환을 위해 tan 사용
        setTurnRightRadians(Math.tan(turnAngle));

        // 각도에 따라 전진 거리 조절
        setAhead(Math.cos(turnAngle) * Point2D.distance(selfX, selfY, navX, navY));
    }

    /**
     * 레이더를 적에게 고정
     */
    private void lockRadar(double scannedAbsBearing) {
        double radarTurn = robocode.util.Utils.normalRelativeAngle(scannedAbsBearing - getRadarHeadingRadians());
        setTurnRadarRightRadians(Double.POSITIVE_INFINITY * radarTurn);
    }

    /**
     * 에너지와 거리 기반 총알 파워 계산
     */
    private double calculateBulletPower(double energy, double distance) {
        enemyDistance = distance;
        return Math.log10(energy) * 325 / (Math.log10(distance) * 97);
    }

    /**
     * 총이 준비되었을 때만 발사
     */
    private void fireIfReady(double bulletPower, double scannedDist) {
        if (bulletPower > 0 && getGunTurnRemaining() == 0) {
            setFire(bulletPower);
        }
    }

    /**
     * Markov 상태 계산
     */
    private int calculateMarkovState(double enemySpeedNow, double prevSpeed, double prevHeading, double newHeading) {
        int acceleration = (int)Math.signum(enemySpeedNow - prevSpeed) + 1;
        int velocityBin = ((int)(8.5 + enemySpeedNow) << 2);
        int headingDelta = (int)(-2 * robocode.util.Utils.normalRelativeAngle(prevHeading - (enemyHeading = newHeading)) /
                                Rules.getTurnRateRadians(prevSpeed) + 2.5);
        int result = acceleration + velocityBin + (headingDelta << 7);
        // 유효 범위 [0, 578]로 클램프
        return Math.max(0, Math.min(result, 579 - 1));
    }

    /**
     * Markov 기반 예측 및 커널 밀도 계산 (최적 타겟팅 bin 반환)
     */
    private int predictAndKernelDensity(int state, double scannedAbsBearing, double scannedDist, double enemySpeedNow, double bulletPower) {
        int enemyHash = Math.abs(enemyId.hashCode()) % ENEMY_HASH_SIZE;
        int[][] table = transitionMatrix[enemyHash];
        int[] aimBins = new int[AIM_RESOLUTION];
        int bestBin = 0;
        for (int i = 0; i <= 127; i++) {
            double predictedX = selfX + Math.sin(scannedAbsBearing) * scannedDist;
            double predictedY = selfY + Math.cos(scannedAbsBearing) * scannedDist;
            int nextState = state;
            int ticks = 1;
            double heading = enemyHeading;
            double velocity = enemySpeedNow;
            int weight = 100;
            do {
                // 방어적 체크: nextState 유효성
                if (nextState < 0 || nextState >= table.length) {
                    break;
                }
                int tableSize = Math.min(STATE_TABLE_SIZE, table[nextState][STATE_TABLE_SIZE]);
                if (tableSize != 0) {
                    nextState = table[nextState][(int)(Math.random() * tableSize)];
                } else {
                    weight = 5;
                }
                heading += ((nextState >> 7) - 2) * Rules.getTurnRateRadians(velocity) / 2;
                velocity = ((nextState >> 2) & 31) - 8;
                predictedX += Math.sin(heading) * velocity;
                predictedY += Math.cos(heading) * velocity;
                Rectangle2D.Double fieldRect = new Rectangle2D.Double(GUN_WALL_PADDING, GUN_WALL_PADDING,
                        fieldWidth - (2 * GUN_WALL_PADDING), fieldHeight - (2 * GUN_WALL_PADDING));
                if (!fieldRect.contains(predictedX, predictedY)) {
                    weight = 1;
                }
            } while (++ticks * Rules.getBulletSpeed(bulletPower) < Point2D.distance(selfX, selfY, predictedX, predictedY));

            for (int offset = -4; offset <= 4; offset++) {
                int bin = ((int)(AIM_RESOLUTION * absoluteBearing(predictedX, predictedY) / (2 * Math.PI)) + offset + AIM_RESOLUTION) % AIM_RESOLUTION;
                aimBins[bin] += weight + 1 / (Math.abs(offset) + 1);
                if (aimBins[bin] > aimBins[bestBin]) {
                    bestBin = bin;
                }
            }
        }
        return bestBin;
    }

    /**
     * 총 조준 (bestBin에 맞춰 총 회전)
     */
    private void aimGun(int bestBin) {
        double gunTurn = robocode.util.Utils.normalRelativeAngle(2 * Math.PI * bestBin / AIM_RESOLUTION - getGunHeadingRadians());
        setTurnGunRightRadians(gunTurn);
    }

    /**
     * Markov 테이블 갱신
     */
    private void updateMarkovTable(int state) {
        int enemyHash = Math.abs(enemyId.hashCode()) % ENEMY_HASH_SIZE;
        int[][] table = transitionMatrix[enemyHash];
        if (getGunHeat() < 0.7) {
            if (enemyMarkovState < 0 || enemyMarkovState >= table.length) {
                return;
            }
            int[] tableBin = table[enemyMarkovState];
            int idx = tableBin[STATE_TABLE_SIZE];
            if (idx >= STATE_TABLE_SIZE) {
                idx = 0; // 가득 차면 처음부터 덮어씀
            }
            tableBin[idx] = state;
            tableBin[STATE_TABLE_SIZE] = idx + 1;
        }
    }
    
    /**
     * 적 상태 정보 갱신
     */
    private void updateEnemyInfo(int state, double enemySpeedNow, double newHeading) {
        enemyMarkovState = state;
        enemySpeed = enemySpeedNow;
        enemyHeading = newHeading;
    }

    /**
     * 적 사망 시 레이더 초기화
     */
    public void onRobotDeath(RobotDeathEvent e) {
        setTurnRadarRightRadians(enemyDistance = Double.POSITIVE_INFINITY);
    }

    /**
     * 절대 각도 계산 (필드 기준)
     */
    private static double absoluteBearing(double targetX, double targetY) {
        return Math.atan2(targetX - selfX, targetY - selfY);
    }
}

// ========================
// === 적 정보 저장 클래스 ===
// ========================
class Enemy {
    double x, y;               // 적 좌표
    double bearing;            // 내 기준 적의 각도(라디안)
    double distance;           // 적과의 거리
    double energy = 100;       // 적 에너지
    double lateralVelocity;    // 적의 횡방향 속도

    /**
     * 적 정보를 최신화 (스캔 이벤트 기반)
     * @param e ScannedRobotEvent
     * @param robot AdvancedRobot
     */
    void update(ScannedRobotEvent e, AdvancedRobot robot) {
        energy = e.getEnergy();
        bearing = e.getBearingRadians() + robot.getHeadingRadians();
        distance = e.getDistance();
        x = robot.getX() + Math.sin(bearing) * distance;
        y = robot.getY() + Math.cos(bearing) * distance;
        lateralVelocity = e.getVelocity() * Math.sin(e.getHeadingRadians() - bearing);
    }
}

// =================
// === Wave 클래스 ===
// =================
class Wave {
    double sourceX, sourceY;      // 총알 발사 위치
    double startBearing;          // 발사 각도
    double startTime;             // 발사 시각
    double bulletVelocity;        // 총알 속도

    /**
     * 현재까지 총알이 이동한 거리 계산
     */
    double distanceTraveled(long currentTime) {
        return (currentTime - startTime) * bulletVelocity;
    }

    /**
     * 내 위치에 총알이 도달했는지 판정
     */
    boolean hasPassedMe(double myX, double myY, long currentTime) {
        return Point2D.distance(sourceX, sourceY, myX, myY) <= distanceTraveled(currentTime) + 18;
    }

    /**
     * 임의 위치에 총알이 도달했는지 판정
     */
    boolean hasPassedPosition(double x, double y, long time) {
        return Point2D.distance(sourceX, sourceY, x, y) <= distanceTraveled(time);
    }
}