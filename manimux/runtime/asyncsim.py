"""Independent simulation control loop; no physical robot or camera lifecycle."""

from __future__ import annotations

import math
import sys
import time
import uuid
from collections.abc import Iterable
from copy import deepcopy
from pathlib import Path
from typing import Any

from manimux.clock import Clock, SystemClock
from manimux.embodiments.asyncsim import AsyncSimClient, AsyncSimRobot, AsyncSimSensor
from manimux.embodiments.asyncsim.robot import AsyncSimCommandRejected
from manimux.policies import PolicyCapabilities
from manimux.policies.worker import PolicyWorkerClient
from manimux.policy_adapter import build_policy_adapter
from manimux.runtime.executors import DirectExecutor, SmoothExecutor
from manimux.runtime.inference import (
    CommitSettings,
    RequestState,
    build_inference_strategy,
    prepare_strategy_chunk,
)
from manimux.runtime.safety import RuntimeState, SafetyGuard
from manimux.runtime.scheduling import RequestScheduler
from manimux.runtime.timeline import ActionTimeline, CommitResult
from manimux.types import (
    ActionChunk,
    ActionContext,
    InferenceResponse,
    ObservationSnapshot,
    RobotCommand,
    RobotState,
    copy_group_vector,
)


class AsyncSimRuntime:
    """Own one AsyncSim session, one timeline, and one canonical command stream."""

    def __init__(
        self, config: dict, *, client: Any = None, clock: Clock | None = None, worker: Any = None,
    ) -> None:
        if config["executor"]["type"] not in {"direct", "smooth"}:
            raise ValueError("AsyncSim currently supports direct and smooth executors")
        if config["inference"]["algorithm"] not in {
            "async", "serial", "act_temporal_ensemble", "rtc", "paint",
        }:
            raise ValueError("AsyncSim inference strategy is not supported")
        if config["policy"]["action_decoding"] != "inline" or config["inference"]["independent_group_decoding"]:
            raise ValueError("AsyncSim requires inline, non-independent action decoding")
        self.config = config
        self.clock = clock or SystemClock()
        self.session_id = f"asyncsim-{uuid.uuid4().hex}"
        self.worker = worker
        self.strategy = build_inference_strategy(config)
        self.scheduler = RequestScheduler(config["inference"])
        self._multi_inflight = self.scheduler.mode == "multi_inflight"
        self.adapter = build_policy_adapter(
            config["robot"], config["policy"], motion_limits=config["executor"]["motion_limits"],
        )
        self.adapter.validate(config["robot"], config["policy"])
        robot = config["robot"]
        options = robot["options"]
        self.client = client or AsyncSimClient(options["endpoint"], env_index=options.get("env_index", 0))
        self.robot = AsyncSimRobot(
            self.client, robot["group_dims"], options["state_keys"],
            state_stream=options.get("state_stream", "proprio.joint_state"),
            env_index=options.get("env_index", 0),
            command_ttl_s=options.get("command_ttl_s", 0.2),
        )
        sensor_specs = [item for item in config["sensors"] if item["driver"] == "asyncsim"]
        if len(sensor_specs) != 1 or len(config["sensors"]) != 1:
            raise ValueError("AsyncSim requires one asyncsim sensor bundle")
        self.sensor = AsyncSimSensor(
            self.client, sensor_specs[0]["options"]["streams"],
            env_index=options.get("env_index", 0),
        )
        self.streams = [self.robot.state_stream, *self.sensor.streams.values()]
        self.observation_streams = config["run"].get("observation_streams", [])
        if not isinstance(self.observation_streams, list) or any(
            not isinstance(name, str) or not name for name in self.observation_streams
        ):
            raise ValueError("run.observation_streams must be a list of stream names")
        self.streams = list(dict.fromkeys([*self.streams, *self.observation_streams]))
        self.instruction_stream = config["run"].get("instruction_stream")
        if self.instruction_stream is not None:
            if not isinstance(self.instruction_stream, str) or not self.instruction_stream:
                raise ValueError("run.instruction_stream must be a non-empty stream name")
            if self.instruction_stream in self.streams:
                raise ValueError("instruction stream must differ from state and camera streams")
            self.streams.append(self.instruction_stream)
        self.dt_ns = round(1_000_000_000 / robot["control_hz"])
        inference = config["inference"]
        self.timeline = ActionTimeline(
            robot["group_dims"],
            max_source_steps=inference["max_chunk_policy_steps"],
            action_start_mode=inference["action_start_mode"],
            hold_last_step=inference["inference_schedule"] == "serial",
        )
        self.executor = (
            SmoothExecutor(config["executor"]["smooth"], self.dt_ns / 1e9)
            if config["executor"]["type"] == "smooth"
            else DirectExecutor(config["executor"]["motion_limits"], self.dt_ns / 1e9)
        )
        limits = config["executor"]["command_safety"]
        self.safety = SafetyGuard(
            robot["group_dims"], (
                config["executor"]["smooth"]["position_limit_abs"]
                if config["executor"]["type"] == "smooth" else None
            ),
            position_lower=limits["position_lower"], position_upper=limits["position_upper"],
            max_velocity=limits["max_velocity"], max_acceleration=limits["max_acceleration"],
            control_dt_s=self.dt_ns / 1e9,
        )
        self._last_command = None
        self._started = False
        self._selected_packets: dict[tuple[str, int], dict[str, Any]] = {}
        self._snapshot_audit: list[dict[str, Any]] = []
        self._instruction: str | None = None
        self._skip_unplanned_commands = bool(config["run"].get("skip_unplanned_commands", False))
        self._simulation_timeline = bool(config["run"].get("simulation_time_timeline", False))
        self._control_period_alignment = config["run"].get("control_period_alignment", "episode")
        if self._control_period_alignment not in {"episode", "plan"}:
            raise ValueError("run.control_period_alignment must be episode or plan")
        if self._control_period_alignment == "plan" and (
            not self._simulation_timeline
            or config["inference"]["inference_schedule"] != "serial"
            or round(config["policy"]["action_dt_s"] * 1e9) != self.dt_ns
            or config["executor"]["type"] != "direct"
        ):
            raise ValueError("plan control alignment requires serial simulation timing with native control intervals")
        if self._simulation_timeline and config["inference"]["algorithm"] not in {"async", "serial", "rtc"}:
            raise ValueError("simulation-time timeline currently requires Async, Serial or RTC inference")
        self._timeline_origin_ns: int | None = None
        self._timeline_now_ns: int | None = None

    def start(
        self, *, seed: int | None = None, recording_directory: str | Path | None = None,
    ) -> dict[str, Any]:
        if self._started:
            raise RuntimeError("AsyncSim runtime already started")
        self.robot.connect()
        try:
            if recording_directory is None:
                episode = self.client.reset(seed=seed)
            else:
                episode = self.client.reset(seed=seed, recording_directory=recording_directory)
            self.robot.reset()
            inference = self.config["inference"]
            self.timeline = ActionTimeline(
                self.robot.group_dims,
                max_source_steps=inference["max_chunk_policy_steps"],
                action_start_mode=inference["action_start_mode"],
                hold_last_step=inference["inference_schedule"] == "serial",
            )
            self._last_command = None
            self._selected_packets = {}
            self._snapshot_audit = []
            self._instruction = None
            self._timeline_origin_ns = None
            self._timeline_now_ns = None
            self.client.subscribe(self.streams)
            self.sensor.start()
            self._started = True
            return episode
        except BaseException:
            try:
                self.close()
            except Exception:
                pass
            raise

    def observe(self) -> ObservationSnapshot:
        if not self._started:
            raise RuntimeError("start the AsyncSim runtime before observing")
        snapshot = self.client.read_snapshot(self.streams)
        if self._simulation_timeline:
            sim_ns = round(snapshot["sim_ts"] * 1_000_000_000)
            if self._timeline_origin_ns is None:
                self._timeline_origin_ns = self.clock.now_ns() - sim_ns
            self._timeline_now_ns = self._timeline_origin_ns + sim_ns
        selections = []
        for stream, entry in snapshot["packets"].items():
            packet = entry.get("packet")
            selections.append({
                "stream": stream,
                "source": "fresh" if packet is not None else "missing",
                "seq": None if packet is None else packet["seq"],
                "age_s": entry.get("age_s"),
                "capture_ts": None if packet is None else packet["capture_ts"],
                "available_ts": None if packet is None else packet.get("available_ts"),
            })
            if packet is not None:
                self._selected_packets[(stream, packet["seq"])] = {
                    "episode_id": packet["episode_id"], "stream": stream,
                    "seq": packet["seq"], "capture_ts": packet["capture_ts"],
                    "available_ts": packet.get("available_ts", packet["capture_ts"]),
                    "payload": None,
                }
        self._snapshot_audit.append({
            "snapshot_id": len(self._snapshot_audit),
            "episode_id": snapshot["episode_id"], "snapshot_ts": snapshot["sim_ts"],
            "selections": selections,
        })
        if self.instruction_stream is not None:
            packet = snapshot["packets"][self.instruction_stream]["packet"]
            if packet is not None:
                if packet["episode_id"] != self.client.episode_id:
                    raise RuntimeError("instruction packet belongs to another episode")
                payload = packet["payload"]
                value = payload.get(self.robot.env_index, payload.get(str(self.robot.env_index)))
                if not isinstance(value, str) or not value.strip():
                    raise ValueError("AsyncSim instruction must be a non-empty string")
                self._instruction = value
        state = self.robot.use_snapshot(snapshot)
        frames = self.sensor.use_snapshot(snapshot)
        if self._simulation_timeline:
            assert self._timeline_origin_ns is not None
            state_packet = snapshot["packets"][self.robot.state_stream]["packet"]
            state.monotonic_ns = self._timeline_origin_ns + round(state_packet["capture_ts"] * 1_000_000_000)
            for name, stream in self.sensor.streams.items():
                if name in frames:
                    packet = snapshot["packets"][stream]["packet"]
                    frames[name].capture_monotonic_ns = (
                        self._timeline_origin_ns + round(packet["capture_ts"] * 1_000_000_000)
                    )
        self.safety.validate_state(state)
        if self._last_command is None:
            self._last_command = copy_group_vector(state.groups)
            self.executor.reset(state)
            self.safety.reset(state)
        stream_packets = {}
        for stream in self.observation_streams:
            packet = snapshot["packets"][stream]["packet"]
            if packet is not None:
                if packet["episode_id"] != self.client.episode_id:
                    raise RuntimeError("observation packet belongs to another episode")
                stream_packets[stream] = packet
        return ObservationSnapshot(state, frames, stream_packets)

    def step(self, chunk: ActionChunk | None = None) -> tuple[ObservationSnapshot, CommitResult | None, dict]:
        observation = self.observe()
        result, ack = self.execute(observation, chunk)
        return observation, result, ack

    def execute(
        self, observation: ObservationSnapshot, chunk: ActionChunk | None = None,
        *, commit_settings: CommitSettings | None = None,
    ) -> tuple[CommitResult | None, dict]:
        now_ns = self._timeline_now_ns if self._simulation_timeline else self.clock.now_ns()
        state = observation.state
        result = None
        if chunk is not None:
            if chunk.action_space != "joint_position":
                raise ValueError("AsyncSim requires a canonical joint_position chunk")
            result = self.timeline.commit(
                chunk, now_ns=now_ns,
                max_plan_age_ns=round(self.config["inference"]["max_plan_age_s"] * 1e9),
                handoff_skip_steps=self.config["inference"]["handoff_skip_steps"],
                current_command=(
                    self._last_command if commit_settings is None else commit_settings.current_command
                ),
                blend_steps=(
                    self.config["inference"]["blend_policy_steps"]
                    if commit_settings is None else commit_settings.blend_steps
                ),
            )
        sample_ns = now_ns
        if self._control_period_alignment == "plan":
            active = self.timeline.active_horizon()
            if active is not None and now_ns >= active.start_time_ns:
                sample_ns = active.start_time_ns + ((now_ns - active.start_time_ns) // active.dt_ns) * active.dt_ns
        horizon = self.timeline.reference_horizon(
            now_ns=sample_ns, dt_ns=self.dt_ns, horizon_steps=self.executor.horizon_steps
        )
        if horizon is None:
            if isinstance(self.executor, SmoothExecutor) and self.executor.braking_tracking:
                command = self.executor.brake_hold(now_ns, state)
            elif isinstance(self.executor, SmoothExecutor):
                held_state = type(state)(copy_group_vector(self._last_command), now_ns, state.sequence)
                command = self.executor.hold(now_ns, held_state)
            else:
                command = RobotCommand(copy_group_vector(self._last_command), now_ns, "hold")
        else:
            command = self.executor.step(now_ns, state, horizon)
        self.safety.validate_command(command)
        ack = self.robot.send_command(command)
        self._last_command = copy_group_vector(command.groups)
        return result, ack

    def _check_capabilities(self, worker: Any) -> None:
        capabilities = getattr(worker, "capabilities", PolicyCapabilities())
        if not isinstance(capabilities, PolicyCapabilities):
            raise TypeError("policy worker returned invalid capabilities")
        missing = self.strategy.required_sampling_modes.difference(capabilities.sampling_modes)
        if missing:
            raise RuntimeError(f"policy backend does not support sampling modes {sorted(missing)}")
        if self._multi_inflight and not capabilities.multi_inflight:
            raise RuntimeError("policy backend does not support multi_inflight")
        expected = self.config["policy"]["expected_backend"]
        if expected is not None:
            def compare(required: dict, actual: dict, path: str) -> None:
                for key, value in required.items():
                    if key not in actual:
                        raise RuntimeError(f"policy backend identity missing {path}.{key}")
                    if isinstance(value, dict):
                        if not isinstance(actual[key], dict):
                            raise RuntimeError(f"policy backend identity mismatch at {path}.{key}")
                        compare(value, actual[key], f"{path}.{key}")
                    elif value is not None and value != actual[key]:
                        raise RuntimeError(f"policy backend identity mismatch at {path}.{key}")
            compare(deepcopy(expected), capabilities.backend_metadata, "backend")

    def run_policy(
        self, *, seed: int | None = None, max_steps: int | None = None,
        timeout_s: float | None = None, recording_directory: str | Path | None = None,
    ) -> dict[str, Any]:
        """Execute ManiMux's worker, adapter, strategy, timeline and executor."""
        worker = self.worker or PolicyWorkerClient(
            self.config["policy"], self.session_id, multi_inflight=self._multi_inflight,
        )
        max_steps = max_steps or self.config["run"]["max_control_steps"]
        if max_steps <= 0:
            raise ValueError("max_steps must be positive")
        if timeout_s is not None and timeout_s <= 0:
            raise ValueError("timeout_s must be positive")
        started_wall = time.monotonic()
        accepted = rejected = 0
        rejection_reasons: list[str] = []
        responses = 0
        command_acks: list[dict[str, Any]] = []
        events: list[dict[str, Any]] = []
        inference_ms = 0.0
        request_seq = 0
        last_submitted = -1
        last_deadline = 0
        in_flight = False
        pending_requests: dict[int, tuple[int, int, RobotState]] = {}
        max_pending_requests = (
            max(2, math.ceil(
                self.config["policy"]["timeout_s"] * self.config["inference"]["observation_hz"]
            ) + 2) if self._multi_inflight else 1
        )
        try:
            worker.start()
            self._check_capabilities(worker)
            episode = self.start(seed=seed, recording_directory=recording_directory)
            self.strategy.reset()
            self.scheduler.reset()
            while True:
                initial = self.client.read_snapshot(self.streams)
                if all(entry["packet"] is not None for entry in initial["packets"].values()):
                    break
                if timeout_s is not None and time.monotonic() - started_wall > timeout_s:
                    raise TimeoutError("AsyncSim initial observation streams did not become available")
                self.clock.sleep_until_ns(self.clock.now_ns() + self.dt_ns)
            control_steps = 0
            loop_ticks = 0
            last_control_slot: int | None = None
            last_control_origin: int | None = None
            health = self.client.health()

            def control_origin_ns() -> int:
                if self._control_period_alignment == "plan":
                    active = self.timeline.active_horizon()
                    if active is not None:
                        return active.start_time_ns
                return self._timeline_origin_ns

            def wait_for_poll() -> None:
                # Wall time only paces polling; missed wall ticks never burst.
                poll_dt_ns = max(1, self.dt_ns // 4) if self._simulation_timeline else self.dt_ns
                self.clock.sleep_until_ns(self.clock.now_ns() + poll_dt_ns)

            def submit_if_ready(observation: ObservationSnapshot, now_ns: int) -> bool:
                nonlocal request_seq, last_submitted, last_deadline, in_flight
                if self._multi_inflight:
                    # Coalesced IPC uploads and server acknowledgements may never
                    # reach this loop. Retire their contexts without losing a live anchor.
                    wall_ns = self.clock.now_ns()
                    for seq, (observation_ns, deadline_ns, _) in list(pending_requests.items()):
                        reason = (
                            "superseded" if seq <= self.timeline.accepted_request_seq
                            else "deadline_exceeded" if wall_ns > deadline_ns else None
                        )
                        if reason is not None:
                            pending_requests.pop(seq)
                            self.strategy.on_response_rejected(InferenceResponse(
                                session_id=self.session_id, request_seq=seq,
                                observation_time_ns=observation_ns, deadline_ns=deadline_ns,
                                finished_time_ns=wall_ns, inference_ms=0.0,
                                raw_action=None, error=reason,
                            ))
                            events.append({"kind": "inference_retired", "request_seq": seq,
                                           "reason": reason})
                    in_flight = bool(pending_requests)
                    if len(pending_requests) >= max_pending_requests:
                        return False
                submission = self.scheduler.build_submission(
                    self.strategy,
                    session_id=self.session_id, request_seq=request_seq + 1, now_ns=now_ns,
                    snapshot=observation, adapter=self.adapter, timeline=self.timeline,
                    request_state=RequestState(
                        in_flight, last_submitted, last_deadline, multi_flight=self._multi_inflight,
                    ),
                    runtime_state=RuntimeState.RUNNING,
                )
                if submission is None:
                    return False
                if self.instruction_stream is not None:
                    if self._instruction is None:
                        raise RuntimeError("AsyncSim instruction stream has no visible packet")
                    submission.request.instruction = self._instruction
                prepared = self.adapter.prepare_request(submission.request)
                if self._simulation_timeline:
                    prepared.deadline_ns = self.clock.now_ns() + round(
                        self.config["policy"]["timeout_s"] * 1_000_000_000
                    )
                worker.submit_latest(prepared)
                self.scheduler.on_submitted(now_ns)
                request_seq = prepared.request_seq
                last_submitted = request_seq
                last_deadline = prepared.deadline_ns
                if not self._multi_inflight:
                    pending_requests.clear()
                request_state = prepared.observation.state
                pending_requests[request_seq] = (
                    prepared.observation_time_ns, prepared.deadline_ns,
                    RobotState(
                        copy_group_vector(request_state.groups),
                        request_state.monotonic_ns, request_state.sequence,
                    ),
                )
                in_flight = True
                events.append({"kind": "inference_submitted", "request_seq": request_seq,
                               "instruction": prepared.instruction,
                               "observation_time_ns": prepared.observation_time_ns,
                               "deadline_ns": prepared.deadline_ns,
                               "measured_groups": {name: values.tolist() for name, values in observation.state.groups.items()},
                               "policy_groups": {name: values.tolist() for name, values in prepared.observation.state.groups.items()},
                               "observation_packets": {name: {key: packet.get(key) for key in (
                                   "episode_id", "seq", "capture_ts", "available_ts",
                               )} for name, packet in prepared.observation.stream_packets.items()},
                               **submission.event_fields})
                return True

            while control_steps < max_steps:
                loop_ticks += 1
                if timeout_s is not None and time.monotonic() - started_wall > timeout_s:
                    raise TimeoutError("AsyncSim policy rollout exceeded its wall time limit")
                if not worker.is_alive:
                    raise RuntimeError("policy worker stopped")
                state = health["episode"]["state"]
                if state not in {"ready", "running", "safe_hold"}:
                    break
                if state == "safe_hold":
                    # Physics and fixed-period task evaluation continue in hold.
                    wait_for_poll()
                    health = self.client.health()
                    continue
                observation = self.observe()
                now_ns = self._timeline_now_ns if self._simulation_timeline else self.clock.now_ns()
                if self._simulation_timeline:
                    origin_ns = control_origin_ns()
                    if origin_ns != last_control_origin:
                        last_control_origin = origin_ns
                        last_control_slot = None
                    slot = (now_ns - origin_ns) // self.dt_ns
                    if last_control_slot is not None and slot <= last_control_slot:
                        wait_for_poll()
                        continue
                    if last_control_slot is not None and slot > last_control_slot + 1:
                        events.append({"kind": "control_periods_skipped",
                                       "count": slot - last_control_slot - 1,
                                       "sim_ts": health["episode"]["sim_ts"]})
                    last_control_slot = slot
                response: InferenceResponse | None = worker.poll()
                chunk = None
                settings = None
                if response is not None:
                    responses += 1
                    inference_ms += response.inference_ms
                    events.append({
                        "kind": "inference_response", "request_seq": response.request_seq,
                        "finished_time_ns": response.finished_time_ns,
                        "inference_ms": response.inference_ms,
                    })
                    same_session = response.session_id == self.session_id
                    if same_session and response.request_seq == last_submitted:
                        in_flight = False
                    pending = (
                        pending_requests.pop(response.request_seq, None) if same_session else None
                    )
                    observation_ns, deadline_ns, observation_state = (
                        pending if pending is not None else (None, 0, None)
                    )
                    stale_request = pending is None or (
                        response.request_seq <= self.timeline.accepted_request_seq
                        if self._multi_inflight else response.request_seq != last_submitted
                    )
                    invalid_reason = (
                        ("stale_session" if not same_session else None)
                        or ("stale_request" if stale_request else None)
                        or response.error
                        or ("observation_mismatch" if response.observation_time_ns != observation_ns else None)
                        or ("deadline_exceeded" if (
                            response.finished_time_ns > deadline_ns
                            or self.clock.now_ns() > deadline_ns
                        ) else None)
                        or ("empty_action" if response.raw_action is None else None)
                    )
                    if invalid_reason is not None:
                        if same_session:
                            self.strategy.on_response_rejected(response)
                        rejected += 1
                        rejection_reasons.append(invalid_reason)
                        events.append({"kind": "inference_rejected", "request_seq": response.request_seq,
                                       "reason": invalid_reason})
                    else:
                        try:
                            chunk = self.adapter.decode_action(
                                response.raw_action,
                                ActionContext(
                                    response.request_seq, response.observation_time_ns,
                                    response.finished_time_ns, execution_time_ns=now_ns,
                                    ik_seed=observation.state,
                                    ik_seed_source="measured_state",
                                    observation_state=observation_state,
                                    max_source_steps=self.config["inference"]["max_chunk_policy_steps"],
                                ),
                            )
                            if chunk.action_space != "joint_position":
                                raise ValueError("policy adapter must return joint_position")
                            chunk = prepare_strategy_chunk(
                                self.strategy, chunk=chunk, response=response, now_ns=now_ns,
                            )
                            settings = self.strategy.commit_settings(
                                response=response, measured=observation.state.groups,
                                last_command=self._last_command,
                            )
                        except (TypeError, ValueError) as exc:
                            self.strategy.on_response_rejected(response)
                            rejected += 1
                            rejection_reasons.append(f"invalid_action:{exc}")
                            events.append({"kind": "plan_rejected", "request_seq": response.request_seq,
                                           "reason": f"invalid_action:{exc}"})
                            chunk = None
                if self._simulation_timeline and chunk is not None:
                    # Decoding can take time; commit against current simulation time.
                    observation = self.observe()
                    now_ns = self._timeline_now_ns
                submitted_early = (
                    submit_if_ready(observation, now_ns)
                    if self._simulation_timeline and chunk is None else False
                )
                if self._skip_unplanned_commands and chunk is None and self.timeline.sample(now_ns) is None:
                    result = None
                else:
                    try:
                        result, ack = self.execute(observation, chunk, commit_settings=settings)
                    except AsyncSimCommandRejected as exc:
                        state = self.client.health()["episode"]["state"]
                        if exc.reason != "episode_not_accepting_commands" or state not in {
                            "succeeded", "failed", "finished", "safe_hold",
                        }:
                            raise
                        events.append({"kind": "command_rejected", "reason": exc.reason,
                                       "episode_state": state})
                        if state == "safe_hold":
                            wait_for_poll()
                            health = self.client.health()
                            continue
                        break
                    if self._simulation_timeline:
                        origin_ns = control_origin_ns()
                        # Only the sampled slot was processed, even if sending took longer.
                        sampled_slot = (now_ns - origin_ns) // self.dt_ns
                        if origin_ns != last_control_origin:
                            last_control_origin = origin_ns
                            last_control_slot = sampled_slot
                        else:
                            last_control_slot = max(last_control_slot, sampled_slot)
                        ack["control_slot"] = sampled_slot
                        if self._control_period_alignment == "plan":
                            ack["control_origin_ns"] = origin_ns
                    command_acks.append(ack)
                    events.append({"kind": "command_ack", **ack})
                    control_steps += 1
                if result is not None and response is not None and chunk is not None:
                    if result.accepted:
                        accepted += 1
                        strategy_fields = self.strategy.on_plan_accepted(
                            chunk=chunk, result=result, response=response,
                            now_ns=(self._timeline_now_ns if self._simulation_timeline else self.clock.now_ns()),
                        )
                        events.append({
                            "kind": "plan_committed", "request_seq": response.request_seq,
                            "plan_id": chunk.plan_id, "trimmed_steps": result.trimmed_steps,
                            "groups": {name: values.tolist() for name, values in chunk.groups.items()},
                            "strategy": strategy_fields,
                        })
                    else:
                        rejected += 1
                        rejection_reasons.append(result.reason)
                        self.strategy.on_response_rejected(response)
                        events.append({"kind": "plan_rejected", "request_seq": response.request_seq,
                                       "plan_id": chunk.plan_id, "reason": result.reason})
                if not submitted_early:
                    now_ns = self._timeline_now_ns if self._simulation_timeline else self.clock.now_ns()
                    submit_if_ready(observation, now_ns)
                for kind, fields in self.strategy.take_runtime_events(step=loop_ticks):
                    events.append({"kind": kind, **fields})
                self.strategy.on_tick(steps=loop_ticks, loop_ms=0.0, control_dt_ns=self.dt_ns)
                health = self.client.health()
                if health["episode"]["state"] not in {"ready", "running", "safe_hold"}:
                    break
                wait_for_poll()
            return {
                "session_id": self.session_id, "episode_id": episode["episode_id"],
                "steps": control_steps, "loop_ticks": loop_ticks, "responses": responses,
                "accepted_plans": accepted, "rejected_plans": rejected,
                "rejection_reasons": rejection_reasons,
                "command_acks": command_acks,
                "asyncsim_result": self.client.result(),
                "asyncsim_metrics": self.client.metrics(),
                "backend_metadata": getattr(worker, "capabilities", PolicyCapabilities()).backend_metadata,
                "inference_time_s": inference_ms / 1000.0,
                "wall_elapsed_s": time.monotonic() - started_wall,
                "audit": {"episode_id": episode["episode_id"], "snapshots": self._snapshot_audit,
                          "actions": command_acks, "manimux_events": events},
                "packets": tuple(self._selected_packets.values()),
            }
        finally:
            cleanup_error = None
            for close in (worker.close, self.close):
                try:
                    close()
                except Exception as exc:
                    if cleanup_error is None:
                        cleanup_error = exc
            if cleanup_error is not None and sys.exc_info()[0] is None:
                raise cleanup_error

    def run(self, chunks: Iterable[ActionChunk | None], *, seed: int | None = None) -> list[dict]:
        """Run a fake-policy sequence; worker submission is added separately."""
        acknowledgements = []
        self.start(seed=seed)
        next_tick = self.clock.now_ns()
        try:
            for chunk in chunks:
                _, _, ack = self.step(chunk)
                acknowledgements.append(ack)
                next_tick += self.dt_ns
                self.clock.sleep_until_ns(next_tick)
            return acknowledgements
        finally:
            self.close()

    def close(self) -> None:
        self._started = False
        self.sensor.close()
        self.robot.close()
