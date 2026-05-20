"""Optional HTTP / WebSocket front-end for the JSON bridge.

Activated by setting `enable_http: true` on `json_bridge_node`.  The two
ports of contact are:

    POST /command   -> body: envelope JSON, returns response JSON
    GET  /status    -> liveness probe
    WS   /events    -> server pushes every JSON event (action feedback +
                       final result) to all connected clients

Requires:  pip install fastapi uvicorn websockets
"""

from __future__ import annotations

import asyncio
import json
from typing import Any

from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import PlainTextResponse
import uvicorn


def run_http_gateway(node: Any, host: str = "0.0.0.0", port: int = 8080) -> None:
    """Blocking call.  Runs FastAPI/uvicorn in the calling thread.

    `node` must expose `handle_json(str)->str`, `add_ws_subscriber(cb)` and
    `remove_ws_subscriber(cb)`.
    """
    app = FastAPI(title="OpenArm JSON API",
                  description="Upper-computer / LLM interface for the OpenArm robot.",
                  version="0.1.0")

    @app.get("/status", response_class=PlainTextResponse)
    async def status() -> str:
        return "ok"

    @app.post("/command")
    async def command(request: Request):
        body = await request.body()
        loop = asyncio.get_event_loop()
        # node.handle_json is synchronous and blocks on rclpy.spin_*; offload.
        out = await loop.run_in_executor(None, node.handle_json, body.decode("utf-8"))
        return json.loads(out)

    @app.websocket("/events")
    async def events(ws: WebSocket):
        await ws.accept()
        loop = asyncio.get_event_loop()
        queue: asyncio.Queue[str] = asyncio.Queue()

        def push(payload: str) -> None:
            loop.call_soon_threadsafe(queue.put_nowait, payload)

        node.add_ws_subscriber(push)
        try:
            while True:
                payload = await queue.get()
                await ws.send_text(payload)
        except WebSocketDisconnect:
            pass
        finally:
            node.remove_ws_subscriber(push)

    config = uvicorn.Config(app, host=host, port=port, log_level="info",
                            access_log=False)
    server = uvicorn.Server(config)
    server.run()
