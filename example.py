import os
import asyncio
from dotenv import load_dotenv

from deepagents import create_deep_agent
from langchain_openai import ChatOpenAI
from mcp.client.sse import sse_client
from mcp.client.session import ClientSession
from langchain_mcp_adapters.tools import load_mcp_tools

load_dotenv()

async def main():
    # API key check
    tavily_key = os.environ.get("TAVILY_API_KEY")
    if not tavily_key:
        print("Error: TAVILY_API_KEY environment variable is missing.")
        return

    # Set up Tavily MCP Server via SSE
    url = f"https://mcp.tavily.com/mcp/?tavilyApiKey={tavily_key}"

    print("Connecting to Tavily MCP Server via SSE...")
    try:
        async with sse_client(url) as (read, write):
            async with ClientSession(read, write) as session:
                await session.initialize()
                print("Connected to Tavily MCP Server!")

                # Load tools from MCP Server
                tools = await load_mcp_tools(session)
                print(f"Loaded {len(tools)} tools from MCP Server: {[t.name for t in tools]}")

                # Create DeepAgent with explicit OpenAI LLM
                llm = ChatOpenAI(model="gpt-4o")
                agent = create_deep_agent(
                    model=llm,
                    tools=tools,
                    system_prompt="You are a helpful company research assistant. Use the provided search tools to find accurate information and write comprehensive analysis reports."
                )

                print("\n--- DeepAgent is ready ---\n")
                
                # Request example
                query = "OpenAI 회사의 최근 행보와 주요 제품들에 대해 깊이 있게 조사하고 분석해줘."
                print(f"User Query: {query}")
                print("\nAgent is thinking...\n")

                # Note: using ainvoke for async execution
                async for chunk in agent.astream({"messages": [("user", query)]}):
                    if "agent" in chunk:
                        print(chunk["agent"]["messages"][0].content)
                    elif "tools" in chunk:
                        for tool_msg in chunk['tools']['messages']:
                            print(f"[Tool Execution] {tool_msg.name}")
                            
    except Exception as e:
        print(f"Error during execution: {e}")
        print("Please ensure your OPENAI_API_KEY and TAVILY_API_KEY are set correctly.")

if __name__ == "__main__":
    asyncio.run(main())
