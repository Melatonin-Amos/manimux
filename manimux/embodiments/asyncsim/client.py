"""Synchronous AsyncSim wire client owned by the simulation runtime."""

from __future__ import annotations

import itertools
from copy import deepcopy
from pathlib import Path
from typing import Any


class AsyncSimRemoteError(RuntimeError):
    def __init__(self, code: str, message: str) -> None:
        self.code = code
        super().__init__(f"AsyncSim {code}: {message}")


class AsyncSimClient:
    def __init__(self, endpoint: str, *, timeout_s: float = 10.0, env_index: int = 0,
                 compression: str | None = "deflate", cache_snapshot_payloads: bool = False) -> None:
        if not endpoint.startswith(("ws://", "wss://")) or timeout_s <= 0:
            raise ValueError("AsyncSim requires a WebSocket endpoint and positive timeout")
        self.endpoint = endpoint
        self.timeout_s = timeout_s
        self.compression = compression
        self.cache_snapshot_payloads = cache_snapshot_payloads
        self._payloads: dict[str, tuple[int, Any]] = {}
        self.wire_received_bytes = 0
        self.payload_reuses = 0
        if isinstance(env_index, bool) or not isinstance(env_index, int) or env_index < 0:
            raise ValueError("env_index must be a non-negative integer")
        self.env_index = env_index
        self.episode_id: str | None = None
        self._connection: Any = None
        self._connection_context: Any = None
        self._ids = itertools.count(1)

    def connect(self) -> None:
        if self._connection is not None:
            return
        try:
            from websockets.sync.client import connect
        except ImportError as exc:
            raise RuntimeError("AsyncSim requires the xpolicylab WebSocket extra") from exc
        context = connect(
            self.endpoint, open_timeout=self.timeout_s, max_size=64 * 1024 * 1024,
            ping_timeout=max(20.0, self.timeout_s),
            compression=self.compression,
        )
        self._connection = context.__enter__()
        self._connection_context = context

    def request(self, op: str, payload: dict[str, Any] | None = None) -> dict[str, Any]:
        if self._connection is None:
            raise RuntimeError("AsyncSim client is not connected")
        import msgpack
        import msgpack_numpy

        request_id = f"manimux-{next(self._ids):08d}"
        wire = {"request_id": request_id, "op": op, "payload": payload or {}, "version": "v1"}
        self._connection.send(msgpack.packb(wire, use_bin_type=True, default=msgpack_numpy.encode))
        wire_response = self._connection.recv(timeout=self.timeout_s)
        self.wire_received_bytes += len(wire_response)
        response = msgpack.unpackb(
            wire_response,
            raw=False, strict_map_key=False, object_hook=msgpack_numpy.decode,
        )
        if response.get("version") != "v1" or response.get("request_id") != request_id:
            raise RuntimeError("AsyncSim response version or request id mismatch")
        if not response.get("ok"):
            error = response.get("error") or {}
            raise AsyncSimRemoteError(error.get("code", "error"), error.get("message", ""))
        return response["payload"]

    def reset(
        self, *, seed: int | None = None, recording_directory: str | Path | None = None,
    ) -> dict[str, Any]:
        payload = {} if seed is None else {"seed": seed}
        if self.env_index:
            payload["env_index"] = self.env_index
        if recording_directory is not None:
            payload["recording_directory"] = str(Path(recording_directory).resolve())
        result = self.request("reset", payload)
        self.episode_id = result["episode_id"]
        self._payloads.clear()
        return result

    def subscribe(self, streams: list[str]) -> None:
        self.request("subscribe", {"streams": streams})

    def read_snapshot(self, streams: list[str]) -> dict[str, Any]:
        request: dict[str, Any] = {"streams": streams}
        if self.cache_snapshot_payloads:
            request["known_payloads"] = {
                name: self._payloads[name][0] for name in streams if name in self._payloads
            }
        result = self.request("read_snapshot", request)
        if result["episode_id"] != self.episode_id:
            raise RuntimeError("AsyncSim snapshot belongs to another episode")
        if self.cache_snapshot_payloads:
            for name, entry in result["packets"].items():
                packet = entry["packet"]
                if packet is None:
                    continue
                if packet.pop("payload_unchanged", False):
                    seq, data = self._payloads[name]
                    if seq != packet["seq"]:
                        raise RuntimeError("cached payload sequence differs from snapshot")
                    self.payload_reuses += 1
                else:
                    data = packet["payload"]
                    self._payloads[name] = (packet["seq"], data)
                # Keep snapshot ownership unchanged: a consumer cannot mutate the cache.
                packet["payload"] = deepcopy(data)
        return result

    def submit_command(self, command: dict[str, Any]) -> dict[str, Any]:
        return self.request("submit_command", {"command": command})

    def health(self) -> dict[str, Any]:
        return self.request("health", {"env_index": self.env_index} if self.env_index else None)

    def result(self) -> dict[str, Any]:
        return self.request("get_result")

    def metrics(self) -> dict[str, Any]:
        return self.request("get_metrics")

    def close(self) -> None:
        connection = self._connection
        if connection is not None:
            try:
                self.request("close")
            except Exception:
                # Closing a broken WebSocket is best effort; keep the original error.
                pass
            finally:
                self._connection = None
                context, self._connection_context = self._connection_context, None
                try:
                    context.__exit__(None, None, None)
                except Exception:
                    pass
        self.episode_id = None
        self._payloads.clear()
