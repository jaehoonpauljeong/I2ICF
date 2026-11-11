"""Synchronous planner adapter using official MCP sessions over Streamable HTTP.

Each request has its own initialized session. Motion requests are never retried
automatically: a lost response can mean that the command already executed.
"""
import asyncio
import copy
from datetime import timedelta
import json
import os
import threading
import time
import uuid

import httpx
from mcp import ClientSession
from mcp.client.streamable_http import streamable_http_client
from common.config import NET_CFG


class MCPTransportError(RuntimeError):
    pass


class MCPROS2Client:
    def __init__(self, world_provider, result_callback=None, url=None, timeout=None):
        self.url = url or os.getenv("MCP_SERVER_URL", f"http://{NET_CFG.edge_host}:{NET_CFG.mcp_port}/mcp")
        self.timeout = float(timeout if timeout is not None else os.getenv("MCP_REQUEST_TIMEOUT_SEC", "10"))
        if not 0 < self.timeout <= 120:
            raise ValueError("MCP_REQUEST_TIMEOUT_SEC must be in (0, 120]")
        self._world_provider = world_provider
        self.result_callback = result_callback
        self._tools = None
        self.lock = threading.Lock()
        self._status = {"transport":"streamable-http", "url":self.url,
                        "connected":False, "last_error":None, "last_success_at":None}

    def status(self):
        with self.lock:
            return copy.deepcopy(self._status)

    async def _exchange(self, name=None, args=None, meta=None):
        async with httpx.AsyncClient(timeout=self.timeout) as http:
            async with streamable_http_client(self.url, http_client=http) as (read, write, _):
                async with ClientSession(read, write, read_timeout_seconds=timedelta(seconds=self.timeout)) as session:
                    await session.initialize()
                    if name is not None:
                        return await session.call_tool(name, args, meta=meta)
                    tools, cursor = [], None
                    while True:
                        page = await session.list_tools(cursor=cursor)
                        tools.extend({"name":t.name,"description":t.description or "",
                                      "inputSchema":t.inputSchema} for t in page.tools)
                        cursor = page.nextCursor
                        if not cursor:
                            return tools

    def _request(self, *args):
        async def bounded():
            return await asyncio.wait_for(self._exchange(*args), self.timeout)
        try:
            result = asyncio.run(bounded())
        except Exception as exc:
            # ExceptionGroup messages alone hide useful connection errors.
            def describe(error):
                children = getattr(error, "exceptions", ())
                return "; ".join(describe(e) for e in children) if children else f"{type(error).__name__}: {error}"
            message = describe(exc)
            with self.lock:
                self._status.update(connected=False, last_error=message)
            raise MCPTransportError(message) from exc
        with self.lock:
            self._status.update(connected=True, last_error=None, last_success_at=time.time())
        return result

    def list_tools(self):
        if self._tools is None:
            self._tools = self._request()
        return copy.deepcopy(self._tools)

    def probe(self):
        self._tools = self._request()
        return self.status()

    def get_ollama_tools_schema(self):
        return [{"type":"function", "function":{"name":t["name"],
                 "description":t["description"], "parameters":t["inputSchema"]}}
                for t in self.list_tools()]

    def call_tool(self, name, arguments=None, *, expected_intent_version=None):
        correlation = str(uuid.uuid4())
        trace = {"transport":"streamable-http", "endpoint":self.url, "method":"tools/call",
                 "correlation_id":correlation, "intent_version":expected_intent_version}
        try:
            reply = self._request(name, arguments or {}, {"ietf127":{
                "correlation_id":correlation, "intent_version":expected_intent_version,
                "expires_at":time.time()+self.timeout}})
            blocks = [b.text for b in reply.content if b.type == "text"]
            result = json.loads("".join(blocks))
            if not isinstance(result, dict) or not isinstance(result.get("ok"), bool):
                raise ValueError("T6 returned an invalid tool result")
            ack = result.get("mcp_server", {})
            if ack.get("correlation_id") != correlation or ack.get("intent_version") != expected_intent_version:
                raise ValueError("T6 context acknowledgement missing/mismatched; update T6 with Edge")
            if reply.isError:
                result["ok"] = False
            trace["server_request_id"] = ack.get("request_id")
            result["mcp_transport"] = trace
            if self.result_callback:
                self.result_callback(name, result, expected_intent_version)
            return result
        except (MCPTransportError, ValueError, TypeError) as exc:
            with self.lock:
                self._status.update(connected=False, last_error=str(exc))
            return {"ok":False, "error":"MCP_TRANSPORT_ERROR", "message":str(exc),
                    "execution_uncertain":True, "mcp_transport":trace}


if __name__ == "__main__":
    client = MCPROS2Client(lambda: {})
    try:
        print(json.dumps({**client.probe(), "tools":[t["name"] for t in client.list_tools()]}, indent=2))
    except MCPTransportError as exc:
        raise SystemExit(f"T6 MCP unavailable: {exc}")
