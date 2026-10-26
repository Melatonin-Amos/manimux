"""Whole-chunk transport and physical completion tracking for AsyncSim."""

from __future__ import annotations

from collections.abc import Mapping
import math
from typing import Any

import numpy as np

from manimux.embodiments.asyncsim.robot import AsyncSimCommandRejected


class NativeChunkExecution:
    """Own the command sequence; never mix this path with robot.send_command."""

    mode = "native_chunk_boundary_replace_v1"

    def __init__(self, client: Any, group_dims: Mapping[str, int], *, action_space: str, ttl_s: float):
        if action_space not in {"joint_position", "eef_pose"}:
            raise ValueError("unsupported native chunk action space")
        if not math.isfinite(ttl_s) or ttl_s <= 0:
            raise ValueError("native chunk TTL must be finite and positive")
        self.client = client
        self.group_dims = dict(group_dims)
        self.action_space = action_space
        self.ttl_s = ttl_s
        self.reset(None)

    def reset(self, episode_id: str | None) -> None:
        self.episode_id = episode_id
        self.command_seq = 0
        self.last_plan_id: str | None = None

    def submit(self, chunk: Any, *, sim_ts: float) -> dict[str, Any]:
        if self.episode_id is None or self.client.episode_id != self.episode_id:
            raise RuntimeError("native chunk session does not match the current episode")
        if chunk.action_space != self.action_space or set(chunk.groups) != set(self.group_dims):
            raise ValueError("chunk action space or groups do not match AsyncSim")
        count = chunk.horizon_steps
        if not 1 <= count <= 4096 or any(
            rows.shape != (count, self.group_dims[name]) or not np.isfinite(rows).all()
            for name, rows in chunk.groups.items()
        ):
            raise ValueError("invalid native action chunk layout")
        if chunk.source_offset_steps or chunk.runtime_trajectory is not None:
            raise ValueError("native chunks cannot trim or resample policy actions")
        commands = [{
            "episode_id": self.episode_id,
            "command_seq": self.command_seq + index + 1,
            "issued_sim_ts": sim_ts,
            "apply_after_sim_ts": sim_ts,
            "expires_at_sim_ts": sim_ts + self.ttl_s,
            "plan_id": chunk.plan_id,
            "action_space": chunk.action_space,
            "groups": {name: rows[index].tolist() for name, rows in chunk.groups.items()},
        } for index in range(count)]
        packet = {"commands": commands, "action_dt_s": chunk.dt_ns / 1e9}
        pause_token = getattr(self.client, "inference_pause_token", None)
        if pause_token is None:
            ack = self.client.request("submit_action_chunk", {"chunk": packet})
        else:
            ack = self.client.request("finish_inference", {
                "token": pause_token, "chunk": packet, "reason": "command_ready",
            })
        if not ack["accepted"]:
            raise AsyncSimCommandRejected(ack["reason"])
        if (ack.get("episode_id") != self.episode_id or ack.get("plan_id") != chunk.plan_id
                or ack.get("command_seq") != self.command_seq + 1 or ack.get("action_count") != count):
            raise RuntimeError("AsyncSim acknowledged a different action chunk")
        self.command_seq += count
        self.last_plan_id = chunk.plan_id
        if pause_token is not None:
            self.client.inference_pause_token = None
        return {**ack, "issued_sim_ts": sim_ts, "action_count": count}

    def complete(self, health: dict[str, Any]) -> bool:
        progress = health.get("action_chunk", {})
        if not progress.get("enabled") or progress.get("mode") != self.mode:
            raise RuntimeError("simulator does not support ordered native action chunks")
        feedback = progress.get("feedback", {})
        if type(feedback.get("physics_step_id")) is not int or feedback["physics_step_id"] < 0:
            raise RuntimeError("native action execution lacks bottom-level physics feedback")
        episode = health.get("episode") or {}
        if self.episode_id is None or episode.get("episode_id") != self.episode_id:
            raise RuntimeError("native execution feedback belongs to another episode")
        if type(progress.get("busy")) is not bool:
            raise RuntimeError("native execution feedback lacks an explicit busy state")
        if progress["busy"]:
            return False
        if self.last_plan_id is None:
            return True
        return (
            progress.get("last_completed_plan_id") == self.last_plan_id
            and progress.get("last_completed_command_seq") == self.command_seq
            and feedback.get("command_seq") == self.command_seq
            and feedback.get("action_completed") is True
            and type(feedback.get("interpolation_ticks")) is int
            and feedback["interpolation_ticks"] > 0
            and feedback.get("interpolation_tick") == feedback["interpolation_ticks"]
        )
