from __future__ import annotations

import time
import unittest
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

from manimux.cli import load_config
from manimux.embodiments.asyncsim import AsyncSimRobot, AsyncSimSensor
from manimux.embodiments.asyncsim.client import AsyncSimRemoteError
from manimux.policies import build_policy_model
from manimux.policies.capabilities import PolicyCapabilities
from manimux.policies.xpolicylab.client import XPolicyLabWsPolicyModel
from manimux.policy_adapter import build_policy_adapter
from manimux.policy_adapter.openwam.arx_x5 import OpenWAMX5Request
from manimux.runtime.asyncsim import AsyncSimRuntime
from manimux.runtime.paint import PaintInferenceRequest
from manimux.runtime.rtc.request import RtcInferenceRequest
from manimux.runtime.train_time_rtc import TrainTimeRtcRequest
from manimux.types import (
    ActionChunk,
    ActionContext,
    InferenceRequest,
    InferenceResponse,
    ObservationSnapshot,
    RobotCommand,
    RobotState,
)


class FakeAsyncSim:
    def __init__(self, clock_ns=lambda: 1_000_000_000) -> None:
        self.episode_id = None
        self.commands = []
        self.streams = []
        self.closed = False
        self.sim_ts = 0.0
        self.clock_ns = clock_ns
        self.instruction = "Pick up the key and insert it."

    def connect(self) -> None:
        self.closed = False

    def reset(self, *, seed=None):
        self.episode_id = "episode-fake"
        self.sim_ts = 0.0
        return {"episode_id": self.episode_id, "seed": seed}

    def subscribe(self, streams):
        self.streams = streams

    def read_snapshot(self, streams):
        assert streams == self.streams
        packets = {
            "proprio.joint_state": {
                "packet": {"episode_id": self.episode_id, "seq": 3,
                           "capture_ts": self.sim_ts, "payload": {0: {
                               "left_joints": [0.0, 0.0], "right_joints": [0.0, 0.0],
                           }}}
            },
            "camera.head.rgb": {
                "packet": {"episode_id": self.episode_id, "seq": 1,
                           "capture_ts": self.sim_ts, "payload": {0: np.zeros((2, 3, 3), dtype=np.uint8)}}
            },
            "language.instruction": {
                "packet": {"episode_id": self.episode_id, "seq": 1,
                           "capture_ts": self.sim_ts, "payload": {0: self.instruction}}
            },
        }
        return {"episode_id": self.episode_id, "sim_ts": self.sim_ts,
                "clock_anchor": {"sim_ts": self.sim_ts, "monotonic_ns": self.clock_ns()},
                "packets": {name: packets[name] for name in streams}}

    def health(self):
        return {"episode": {"sim_ts": self.sim_ts, "state": "running"},
                "canonical_group_dims": {"left_arm": 2, "right_arm": 2}}

    def submit_command(self, command):
        self.commands.append(command)
        return {"accepted": True, "reason": "accepted", "plan_id": command["plan_id"],
                "command_seq": command["command_seq"]}

    def result(self):
        return {"episode_id": self.episode_id, "sim_ts": self.sim_ts, "state": "running",
                "result": {"action_count": len(self.commands), "success_rate": 0.0, "score": 0.0}}

    def metrics(self):
        return {"episode_id": self.episode_id, "sim_ts": self.sim_ts,
                "runtime": {"ticks": len(self.commands)}}

    def close(self):
        self.closed = True


class FixedClock:
    def __init__(self):
        self.time_ns = 1_000_000_000

    def now_ns(self):
        return self.time_ns

    def sleep_until_ns(self, target_ns):
        self.time_ns = target_ns


class FakeWorker:
    def __init__(self, clock, capabilities=None):
        self.clock = clock
        self.capabilities = capabilities or PolicyCapabilities()
        self.is_alive = False
        self.submitted = []
        self._pending = None

    def start(self):
        self.is_alive = True

    def submit_latest(self, request):
        self.submitted.append(request)
        self._pending = request

    def poll(self):
        request, self._pending = self._pending, None
        if request is None:
            return None
        actions = {name: np.ones((4, 2)) for name in request.observation.state.groups}
        return InferenceResponse(
            request.session_id, request.request_seq, self.clock.now_ns(), 1.0,
            {"format": "joint", "actions": actions,
             "action_semantics": "absolute_joint_position"},
            observation_time_ns=request.observation_time_ns,
        )

    def close(self):
        self.is_alive = False


class StaleWorker(FakeWorker):
    def poll(self):
        response = super().poll()
        if response is not None:
            response.session_id = "another-session"
        return response


class DelayedWorker(FakeWorker):
    def __init__(self, clock):
        super().__init__(clock)
        self._wait = 0

    def submit_latest(self, request):
        super().submit_latest(request)
        self._wait = 2

    def poll(self):
        if self._pending is not None and self._wait:
            self._wait -= 1
            return None
        return super().poll()


class SamplingWorker(FakeWorker):
    def __init__(self, clock, mode):
        super().__init__(clock, PolicyCapabilities(sampling_modes=frozenset({mode})))
        self.mode = mode


class StreamingWorker(FakeWorker):
    def __init__(self, clock, replies=None, modes=frozenset({"default"})):
        super().__init__(clock, PolicyCapabilities(sampling_modes=modes, multi_inflight=True))
        self.replies = replies or {}
        self.poll_ticks = 0

    def submit_latest(self, request):
        self.submitted.append(request)

    def poll(self):
        self.poll_ticks += 1
        reply = self.replies.get(self.poll_ticks)
        if reply is None:
            return None
        seq, overrides = reply
        request = next(item for item in self.submitted if item.request_seq == seq)
        actions = {name: np.ones((4, len(values)))
                   for name, values in request.observation.state.groups.items()}
        return replace(InferenceResponse(
            request.session_id, seq, self.clock.now_ns(), 1.0,
            {"format": "joint", "actions": actions,
             "action_semantics": "absolute_joint_position"},
            observation_time_ns=request.observation_time_ns,
            deadline_ns=request.deadline_ns,
        ), **overrides)


def streaming_config():
    configured = config()
    configured["policy"]["adapter"]["type"] = "manimux.policy_adapter.joint:JointAdapter"
    configured["inference"].update(
        inference_schedule="multi_inflight", request_trigger="continuous", observation_hz=50,
    )
    return configured


def config():
    source = Path(__file__).resolve().parents[2] / "manimux/configs/experiments/asyncsim/fake_joint.yaml"
    return load_config(source)


class AsyncSimRuntimeTests(unittest.TestCase):
    def test_retired_environment_slot_is_rejected(self):
        configured = config()
        configured["robot"]["options"]["env_index"] = 1
        with self.assertRaisesRegex(ValueError, "only env_index=0"):
            AsyncSimRuntime(configured)



    def test_multi_inflight_accepts_valid_earlier_response_and_rejects_older_plan(self):
        clock = FixedClock()
        worker = StreamingWorker(clock, {4: (1, {}), 6: (4, {}), 7: (2, {})})
        run = AsyncSimRuntime(
            streaming_config(), client=FakeAsyncSim(), clock=clock, worker=worker,
        ).run_policy(max_steps=8)
        events = run["audit"]["manimux_events"]
        self.assertEqual([event["request_seq"] for event in events
                          if event["kind"] == "plan_committed"], [1, 4])
        self.assertIn("stale_request", run["rejection_reasons"])
        self.assertTrue(any(event["kind"] == "inference_retired"
                            and event["request_seq"] == 2 and event["reason"] == "superseded"
                            for event in events))
        self.assertEqual(len(worker.submitted), 8)

    def test_multi_inflight_checks_each_requests_deadline(self):
        clock = FixedClock()
        configured = streaming_config()
        configured["policy"]["timeout_s"] = .05
        worker = StreamingWorker(clock, {4: (1, {})})
        run = AsyncSimRuntime(
            configured, client=FakeAsyncSim(), clock=clock, worker=worker,
        ).run_policy(max_steps=5)
        self.assertEqual(run["accepted_plans"], 0)
        self.assertIn("deadline_exceeded", run["rejection_reasons"])
        self.assertGreater(worker.submitted[2].deadline_ns, clock.time_ns - 40_000_000)

    def test_old_session_cannot_consume_current_requests_context(self):
        clock = FixedClock()
        worker = StreamingWorker(clock, {3: (1, {"session_id": "old-episode"}), 4: (1, {})})
        run = AsyncSimRuntime(
            streaming_config(), client=FakeAsyncSim(), clock=clock, worker=worker,
        ).run_policy(max_steps=5)
        self.assertIn("stale_session", run["rejection_reasons"])
        self.assertEqual(run["accepted_plans"], 1)

    def test_multi_inflight_preserves_request_observation_during_overlapping_uploads(self):
        clock = FixedClock()
        worker = StreamingWorker(clock, {12: (1, {})})
        runtime = AsyncSimRuntime(
            streaming_config(), client=FakeAsyncSim(), clock=clock, worker=worker,
        )
        with patch.object(runtime.adapter, "decode_action", wraps=runtime.adapter.decode_action) as decode:
            run = runtime.run_policy(max_steps=13)
        self.assertEqual(run["accepted_plans"], 1)
        context = decode.call_args.args[1]
        self.assertEqual(context.observation_state.monotonic_ns,
                         worker.submitted[0].observation_time_ns)
        self.assertIsNot(context.observation_state, worker.submitted[0].observation.state)

    def test_multi_inflight_bounds_contexts_when_simulation_outpaces_wall_time(self):
        class FastSimClock(FixedClock):
            def sleep_until_ns(self, target_ns):
                self.time_ns += 1_000_000

        class FastAsyncSim(FakeAsyncSim):
            def read_snapshot(self, streams):
                self.sim_ts += .02
                return super().read_snapshot(streams)

        configured = streaming_config()
        configured["run"]["simulation_time_timeline"] = True
        configured["policy"]["timeout_s"] = .1
        clock = FastSimClock()
        worker = StreamingWorker(clock)
        run = AsyncSimRuntime(
            configured, client=FastAsyncSim(clock.now_ns), clock=clock, worker=worker,
        ).run_policy(max_steps=20)
        self.assertEqual(run["steps"], 20)
        self.assertEqual(len(worker.submitted), 7)

    def test_multi_inflight_retires_unreturned_uploads_after_timeout(self):
        clock = FixedClock()
        configured = streaming_config()
        configured["policy"]["timeout_s"] = .05
        worker = StreamingWorker(clock)
        run = AsyncSimRuntime(
            configured, client=FakeAsyncSim(), clock=clock, worker=worker,
        ).run_policy(max_steps=10)
        retired = [event for event in run["audit"]["manimux_events"]
                   if event["kind"] == "inference_retired"]
        self.assertTrue(retired)
        self.assertTrue(all(event["reason"] == "deadline_exceeded" for event in retired))
        self.assertEqual(len(worker.submitted), 10)

    def test_multi_inflight_requires_backend_capability_before_reset(self):
        clock = FixedClock()
        backend = FakeAsyncSim()
        worker = FakeWorker(clock)
        with self.assertRaisesRegex(RuntimeError, "multi_inflight"):
            AsyncSimRuntime(
                streaming_config(), client=backend, clock=clock, worker=worker,
            ).run_policy(max_steps=2)
        self.assertIsNone(backend.episode_id)
        self.assertFalse(worker.submitted)

    def test_transport_selection_keeps_serial_single_inflight(self):
        for mode, multi in (("serial", False), ("multi_inflight", True)):
            with self.subTest(schedule=mode):
                clock = FixedClock()
                configured = streaming_config()
                configured["inference"]["inference_schedule"] = mode
                worker = StreamingWorker(clock, {2: (1, {})})
                with patch("manimux.runtime.asyncsim.PolicyWorkerClient", return_value=worker) as factory:
                    runtime = AsyncSimRuntime(configured, client=FakeAsyncSim(), clock=clock)
                    runtime.run_policy(max_steps=2)
                self.assertEqual(factory.call_args.kwargs["multi_inflight"], multi)

    def test_rtc_multi_inflight_preserves_condition_reference_checks(self):
        clock = FixedClock()
        configured = streaming_config()
        configured["inference"].update(algorithm="rtc", request_trigger="algorithm")
        configured["inference"]["rtc"].update(
            initial_delay_policy_steps=1, min_execute_policy_steps=1,
        )
        worker = StreamingWorker(clock, {4: (1, {}), 5: (2, {})}, frozenset({"rtc"}))
        runtime = AsyncSimRuntime(configured, client=FakeAsyncSim(), clock=clock, worker=worker)
        run = runtime.run_policy(max_steps=6)
        self.assertEqual(run["accepted_plans"], 1)
        self.assertTrue(any("rtc_condition_plan_replaced" in reason
                            for reason in run["rejection_reasons"]))

    def test_x5_request_context_survives_eight_entry_adapter_cache(self):
        class PoseModel:
            base_frame = "base_link"
            tcp_frame = "link6"

            def fk(self, q):
                pose = np.eye(4)
                pose[:3, 3] = q[:3]
                return pose

            def ik(self, target, seed, *, fixed_coordinates):
                joints = seed.copy()
                joints[:3] = target[:3, 3]
                joints[-1] = fixed_coordinates["gripper"]
                return SimpleNamespace(converged=True, joints=joints)

        base = Path(__file__).resolve().parents[2] / "manimux/configs/experiments/asyncsim"
        for model, filename in (
            ("xr1", "robodojo_xr1_insert_tubes_default_serial.yaml"),
            ("openwam", "robodojo_openwam_insert_key.yaml"),
        ):
            with self.subTest(model=model):
                configured = load_config(base / filename)
                models = {name: PoseModel() for name in configured["robot"]["group_dims"]}
                adapter = build_policy_adapter(
                    configured["robot"], configured["policy"],
                    kinematics=SimpleNamespace(models=models),
                )
                original = RobotState({name: np.r_[[.1, .2, .3], [0., 0., 0.], .2]
                                       for name in models}, 100, 1)
                poses = {"left_ee_pose": [.1, .2, .3, 1., 0., 0., 0.],
                         "right_ee_pose": [.1, .2, .3, 1., 0., 0., 0.]}
                snapshot = ObservationSnapshot(original, stream_packets={
                    "proprio.ee_pose": {"payload": {0: poses}},
                })
                request = InferenceRequest("test", 1, 100, 10**10, snapshot)
                adapter.prepare_request(request)
                for seq in range(2, 20):
                    newer = replace(snapshot, state=RobotState(
                        {name: np.r_[[.8, .7, .6], [0., 0., 0.], .9] for name in models},
                        seq * 100, seq,
                    ))
                    adapter.prepare_request(replace(
                        request, request_seq=seq, observation_time_ns=seq * 100, observation=newer,
                    ))
                self.assertNotIn(1, adapter._anchors)
                context = ActionContext(1, 100, 200, observation_state=original)
                if model == "xr1":
                    raw = np.zeros((1, 60))
                    raw[:, 6] = .1
                    raw[:, 14] = .2
                    chunk = adapter.decode_action(raw, context)
                    np.testing.assert_allclose(chunk.groups["left_arm"][0],
                                               np.r_[original.groups["left_arm"][:6], .3])
                    np.testing.assert_allclose(chunk.groups["right_arm"][0, -1], .4)
                else:
                    raw = {"format": "pose", "actions": {
                        name: np.tile(np.r_[poses[f"{name.removesuffix('_arm')}_ee_pose"], .25],
                                      (configured["policy"]["horizon_policy_steps"], 1))
                        for name in models
                    }}
                    chunk = adapter.decode_action(raw, context)
                    self.assertEqual(chunk.horizon_steps, 32)
                    np.testing.assert_allclose(chunk.groups["left_arm"][:, -1], .25)
                with self.assertRaisesRegex(ValueError, "anchor"):
                    adapter.decode_action(raw, replace(context, observation_time_ns=101))

    def test_openwam_request_subclass_preserves_model_state_in_streaming_wire_path(self):
        class WireClient:
            def __init__(self):
                self.uploads = []

            def submit_infer(self, observation, *, sampling):
                self.uploads.append((observation, sampling))
                return "request-1"

        configured = streaming_config()
        model = XPolicyLabWsPolicyModel(configured["policy"])
        wire = WireClient()
        model._client = wire
        model._session_id = "test"
        snapshot = ObservationSnapshot(RobotState({"left_arm": np.zeros(2),
                                                   "right_arm": np.zeros(2)}, 100, 1))
        model._camera_map = {}
        state = {"left_ee_pose": np.array([0., 0., 0., 1., 0., 0., 0.])}
        request = OpenWAMX5Request("test", 1, 100, 10**10, snapshot, model_state=state)
        self.assertTrue(model.submit(request))
        np.testing.assert_array_equal(wire.uploads[0][0]["state"]["left_ee_pose"],
                                      state["left_ee_pose"])
        self.assertEqual(wire.uploads[0][1], {"mode": "default"})
        train_time = TrainTimeRtcRequest("test", 2, 100, 10**10, snapshot)
        with self.assertRaisesRegex(ValueError, "unsupported multi_inflight"):
            model.submit(train_time)
        self.assertEqual(len(wire.uploads), 1)

    def test_xr1_multi_inflight_config_keeps_checkpoint_and_control_contract(self):
        base = Path(__file__).resolve().parents[2] / "manimux/configs/experiments/asyncsim"
        serial = load_config(base / "robodojo_xr1_insert_tubes_default_serial.yaml")
        multi = load_config(base / "robodojo_xr1_insert_tubes_default_multi_inflight.yaml")
        self.assertEqual(serial["inference"]["algorithm"], "serial")
        self.assertEqual(serial["inference"]["inference_schedule"], "serial")
        self.assertEqual(multi["inference"]["algorithm"], "async")
        self.assertEqual(multi["inference"]["inference_schedule"], "multi_inflight")
        self.assertEqual(multi["inference"]["request_trigger"], "continuous")
        self.assertEqual(multi["inference"]["observation_hz"], 25)
        self.assertEqual(multi["policy"], serial["policy"])
        self.assertEqual(multi["policy_server"], serial["policy_server"])
        self.assertEqual(multi["robot"], serial["robot"])

    def test_real_streaming_worker_and_websocket_client(self):
        from threading import Thread

        from websockets.exceptions import ConnectionClosed
        from websockets.sync.server import serve

        from manimux.policies.xpolicylab.ws_client import pack_frame, unpack_frame

        uploads = []

        def handle(connection):
            try:
                for raw in connection:
                    frame = unpack_frame(raw)
                    kind = frame["message_type"]
                    if kind == "hello":
                        reply_type = "hello_ack"
                        payload = {"capabilities": {"sampling_modes": ["default"],
                                                    "multi_inflight": True}}
                    elif kind == "reset":
                        reply_type, payload = "reset_result", {}
                    else:
                        self.assertEqual(kind, "infer")
                        uploads.append(frame)
                        time.sleep(.05)
                        reply_type = "infer_result"
                        payload = {"actions": [
                            frame["payload"]["observation"]["state"]
                            for _ in range(4)
                        ]}
                    connection.send(pack_frame({**frame, "message_type": reply_type,
                                                "payload": payload}))
            except ConnectionClosed:
                pass

        configured = streaming_config()
        configured["policy"].update(worker="xpolicylab_ws", timeout_s=3)
        configured["policy"]["options"].update(action_format="joint", request_timeout_s=3)
        configured["inference"]["max_plan_age_s"] = 3
        with serve(handle, "127.0.0.1", 0, max_size=None) as server:
            configured["policy"]["options"]["server"] = (
                f"ws://127.0.0.1:{server.socket.getsockname()[1]}"
            )
            thread = Thread(target=server.serve_forever, daemon=True)
            thread.start()
            try:
                run = AsyncSimRuntime(configured, client=FakeAsyncSim(time.monotonic_ns)).run_policy(
                    max_steps=12, timeout_s=5,
                )
            finally:
                server.shutdown()
                thread.join(timeout=2)
        self.assertGreater(run["accepted_plans"], 0, run["rejection_reasons"])
        self.assertGreater(len(uploads), 1)
        self.assertTrue(all(frame["payload"]["latest_only"] for frame in uploads))
        submitted_seq = -1
        overlapping_result = False
        for event in run["audit"]["manimux_events"]:
            if event["kind"] == "inference_submitted":
                submitted_seq = event["request_seq"]
            elif event["kind"] == "plan_committed":
                overlapping_result |= event["request_seq"] < submitted_seq
        self.assertTrue(overlapping_result)

    def test_reset_error_survives_cleanup_failure(self):
        class FailedReset(FakeAsyncSim):
            def reset(self, *, seed=None, recording_directory=None):
                raise AsyncSimRemoteError("unstable_layout", "saved scene is unstable")

            def close(self):
                super().close()
                raise ConnectionError("connection already closed")

        backend = FailedReset()
        with self.assertRaises(AsyncSimRemoteError) as caught:
            clock = FixedClock()
            AsyncSimRuntime(config(), client=backend, clock=clock, worker=FakeWorker(clock)).run_policy(seed=2)
        self.assertEqual(caught.exception.code, "unstable_layout")
        self.assertTrue(backend.closed)

    def test_terminal_episode_rejection_preserves_result(self):
        class TerminatingAsyncSim(FakeAsyncSim):
            state = "running"

            def health(self):
                health = super().health()
                health["episode"]["state"] = self.state
                return health

            def submit_command(self, command):
                if self.commands:
                    self.state = "failed"
                    return {"accepted": False, "reason": "episode_not_accepting_commands"}
                return super().submit_command(command)

            def result(self):
                result = super().result()
                result["state"] = self.state
                return result

        backend = TerminatingAsyncSim()
        clock = FixedClock()
        configured = config()
        configured["policy"]["adapter"]["type"] = "manimux.policy_adapter.joint:JointAdapter"
        run = AsyncSimRuntime(configured, client=backend, clock=clock, worker=FakeWorker(clock)).run_policy(max_steps=4)
        self.assertEqual(run["asyncsim_result"]["state"], "failed")
        self.assertEqual(len(run["command_acks"]), 1)
        self.assertTrue(any(event["kind"] == "command_rejected" for event in run["audit"]["manimux_events"]))

    def test_delayed_initial_camera_waits_before_inference(self):
        class DelayedCamera(FakeAsyncSim):
            reads = 0

            def read_snapshot(self, streams):
                self.reads += 1
                snapshot = super().read_snapshot(streams)
                if self.reads <= 2:
                    snapshot["packets"]["camera.head.rgb"]["packet"] = None
                return snapshot

        backend = DelayedCamera()
        clock = FixedClock()
        worker = FakeWorker(clock)
        configured = config()
        configured["policy"]["adapter"]["type"] = "manimux.policy_adapter.joint:JointAdapter"
        result = AsyncSimRuntime(configured, client=backend, clock=clock, worker=worker).run_policy(max_steps=2)
        self.assertGreaterEqual(backend.reads, 4)
        self.assertGreaterEqual(result["accepted_plans"], 1)
        self.assertEqual(len(worker.submitted[0].observation.frames), 1)

    def test_slow_simulation_retains_chunk_with_wall_clock_deadlines(self):
        class SlowClock(FixedClock):
            def sleep_until_ns(self, target_ns):
                self.time_ns = max(self.time_ns, target_ns)

        class SlowAsyncSim(FakeAsyncSim):
            def __init__(self, clock):
                super().__init__(clock_ns=clock.now_ns)
                self.clock = clock

            def read_snapshot(self, streams):
                self.clock.time_ns += 200_000_000
                self.sim_ts += 0.02
                return super().read_snapshot(streams)

        clock = SlowClock()
        backend = SlowAsyncSim(clock)
        worker = FakeWorker(clock)
        configured = config()
        configured["run"]["simulation_time_timeline"] = True
        configured["inference"]["inference_schedule"] = "single_inflight"
        configured["inference"]["refill_threshold_s"] = 0.02
        configured["policy"]["adapter"]["type"] = "manimux.policy_adapter.joint:JointAdapter"
        run = AsyncSimRuntime(configured, client=backend, clock=clock, worker=worker).run_policy(max_steps=8)
        planned = [item["plan_id"] for item in backend.commands if item["plan_id"].startswith("joint-")]
        self.assertGreater(len(planned), len(set(planned)), planned)
        self.assertGreater(run["accepted_plans"], 1)
        self.assertTrue(all(request.deadline_ns > request.observation_time_ns for request in worker.submitted))
        events = run["audit"]["manimux_events"]
        overlap = False
        for index, event in enumerate(events):
            if event["kind"] != "inference_submitted" or event["request_seq"] <= 1:
                continue
            response_index = next((offset for offset in range(index + 1, len(events))
                                   if events[offset]["kind"] == "inference_response"
                                   and events[offset]["request_seq"] == event["request_seq"]), None)
            if response_index is None:
                continue
            overlap |= any(item["kind"] == "command_ack" and item["plan_id"].startswith("joint-")
                           for item in events[index + 1:response_index])
        self.assertTrue(overlap)

    def test_serial_executes_last_row_before_next_inference(self):
        class AdvancingClock(FixedClock):
            def sleep_until_ns(self, target_ns):
                self.time_ns = max(self.time_ns, target_ns)

        class AdvancingAsyncSim(FakeAsyncSim):
            def read_snapshot(self, streams):
                clock.time_ns += 200_000_000
                self.sim_ts = round(self.sim_ts + 0.04, 6)
                return super().read_snapshot(streams)

        class RampWorker(DelayedWorker):
            def poll(self):
                response = super().poll()
                if response is not None:
                    for values in response.raw_action["actions"].values():
                        values[:, 0] = 10 * response.request_seq + np.arange(4)
                return response

        clock = AdvancingClock()
        backend = AdvancingAsyncSim(clock_ns=clock.now_ns)
        configured = config()
        configured["run"].update(simulation_time_timeline=True, skip_unplanned_commands=True)
        configured["robot"]["control_hz"] = 25
        configured["policy"]["action_dt_s"] = 0.04
        configured["policy"]["adapter"]["type"] = "manimux.policy_adapter.joint:JointAdapter"
        configured["inference"]["inference_schedule"] = "serial"
        runtime = AsyncSimRuntime(
            configured, client=backend, clock=clock, worker=RampWorker(clock),
        )
        run = runtime.run_policy(max_steps=8)
        np.testing.assert_allclose(
            [command["groups"]["left_arm"][0] for command in backend.commands],
            [10, 11, 12, 13, 20, 21, 22, 23],
        )
        pending = False
        for event in run["audit"]["manimux_events"]:
            if event["kind"] == "inference_submitted":
                pending = True
            elif event["kind"] == "inference_response":
                pending = False
            elif event["kind"] == "command_ack":
                self.assertFalse(pending, "serial inference overlapped a planned command")
        self.assertEqual(run["accepted_plans"], 2)

    def test_serial_rpc_latency_preserves_every_chunk_row(self):
        class RpcAsyncSim(FakeAsyncSim):
            def advance(self, dt_s):
                self.sim_ts = round(self.sim_ts + dt_s, 6)
                clock.time_ns += round(dt_s * 1e9)

            def read_snapshot(self, streams):
                self.advance(.004)
                return super().read_snapshot(streams)

            def health(self):
                self.advance(.032)
                return super().health()

            def submit_command(self, command):
                self.advance(.004)
                return super().submit_command(command)

        class RampWorker(DelayedWorker):
            def poll(self):
                response = super().poll()
                if response is not None:
                    gripper = np.ones(chunk_steps)
                    gripper[1] = 0.0
                    response.raw_action["actions"] = {
                        name: np.column_stack((
                            100 * response.request_seq + np.arange(chunk_steps), gripper,
                        ))
                        for name in response.raw_action["actions"]
                    }
                return response

        for chunk_steps in (30, 32):
            for alignment in ("episode", "plan"):
                with self.subTest(chunk_steps=chunk_steps, alignment=alignment):
                    clock = FixedClock()
                    backend = RpcAsyncSim(clock_ns=clock.now_ns)
                    configured = config()
                    configured["run"].update(
                        simulation_time_timeline=True, skip_unplanned_commands=True,
                        control_period_alignment=alignment,
                    )
                    configured["robot"]["control_hz"] = 25
                    configured["policy"].update(action_dt_s=.04, horizon_policy_steps=chunk_steps)
                    configured["policy"]["adapter"]["type"] = "manimux.policy_adapter.joint:JointAdapter"
                    configured["inference"]["inference_schedule"] = "serial"
                    worker = RampWorker(clock)
                    runtime = AsyncSimRuntime(configured, client=backend, clock=clock, worker=worker)
                    run = runtime.run_policy(max_steps=2 * chunk_steps, timeout_s=5)
                    for name in backend.commands[0]["groups"]:
                        np.testing.assert_allclose(
                            [item["groups"][name][0] for item in backend.commands],
                            np.concatenate([100 * seq + np.arange(chunk_steps) for seq in (1, 2)]),
                        )
                        np.testing.assert_allclose(
                            [item["groups"][name][1] for item in backend.commands],
                            np.tile(np.where(np.arange(chunk_steps) == 1, 0.0, 1.0), 2),
                        )
                    self.assertEqual(run["accepted_plans"], 2)
                    for offset in (0, chunk_steps):
                        times = [item["issued_sim_ts"] for item in
                                 backend.commands[offset:offset + chunk_steps]]
                        np.testing.assert_allclose(np.diff(times), .04)
                    second_request_sim_ns = worker.submitted[1].observation_time_ns - runtime._timeline_origin_ns
                    self.assertGreaterEqual(
                        second_request_sim_ns,
                        round((backend.commands[0]["issued_sim_ts"] + chunk_steps * .04) * 1e9),
                    )
                    pending = False
                    for event in run["audit"]["manimux_events"]:
                        if event["kind"] == "inference_submitted":
                            self.assertFalse(pending)
                            pending = True
                        elif event["kind"] == "inference_response":
                            pending = False
                        elif event["kind"] == "command_ack":
                            self.assertFalse(pending, "serial inference overlapped a planned command")

    def test_control_slots_track_sampling_not_later_issue_time(self):
        class AdvancingSim(FakeAsyncSim):
            def read_snapshot(self, streams):
                self.sim_ts = round(self.sim_ts + .004, 6)
                return super().read_snapshot(streams)

        clock = FixedClock()
        backend = AdvancingSim(clock_ns=clock.now_ns)
        configured = config()
        configured["run"]["simulation_time_timeline"] = True
        configured["robot"]["control_hz"] = 25
        configured["policy"]["action_dt_s"] = .04
        configured["policy"]["adapter"]["type"] = "manimux.policy_adapter.joint:JointAdapter"
        runtime = AsyncSimRuntime(configured, client=backend, clock=clock, worker=FakeWorker(clock))
        send = runtime.robot.send_command

        def later_issue(command):
            ack = send(command)
            # A later issue timestamp must not claim an unsampled control slot.
            for key in ("issued_sim_ts", "apply_after_sim_ts", "expires_at_sim_ts"):
                ack[key] += .04
            return ack

        runtime.robot.send_command = later_issue
        run = runtime.run_policy(max_steps=8, timeout_s=5)
        self.assertEqual(
            [ack["control_slot"] for ack in run["command_acks"]],
            [round(command["issued_sim_ts"] * 1e9) // runtime.dt_ns for command in backend.commands],
        )

    def test_simulation_control_slots_are_unique_when_wall_time_is_slow(self):
        class SlowSim(FakeAsyncSim):
            def read_snapshot(self, streams):
                clock.time_ns += 200_000_000
                self.sim_ts = round(self.sim_ts + .004, 6)
                return super().read_snapshot(streams)

        clock = FixedClock()
        backend = SlowSim(clock_ns=clock.now_ns)
        configured = config()
        configured["run"]["simulation_time_timeline"] = True
        configured["robot"]["control_hz"] = 25
        configured["policy"]["action_dt_s"] = .04
        configured["policy"]["adapter"]["type"] = "manimux.policy_adapter.joint:JointAdapter"
        run = AsyncSimRuntime(configured, client=backend, clock=clock,
                              worker=FakeWorker(clock)).run_policy(max_steps=8)
        slots = [ack["control_slot"] for ack in run["command_acks"]]
        self.assertEqual(len(slots), len(set(slots)))
        self.assertGreater(run["loop_ticks"], run["steps"])
        self.assertLess(backend.sim_ts, .4)

    def test_decode_retains_request_feedback_and_uses_response_feedback_for_ik(self):
        class ChangingFeedback(FakeAsyncSim):
            reads = 0

            def read_snapshot(self, streams):
                self.reads += 1
                snapshot = super().read_snapshot(streams)
                state = snapshot["packets"]["proprio.joint_state"]["packet"]["payload"][0]
                for name in ("left_joints", "right_joints"):
                    state[name] = [self.reads * .01] * 2
                return snapshot

        clock = FixedClock()
        configured = config()
        configured["policy"]["adapter"]["type"] = "manimux.policy_adapter.joint:JointAdapter"
        worker = FakeWorker(clock)
        runtime = AsyncSimRuntime(configured, client=ChangingFeedback(clock_ns=clock.now_ns),
                                  clock=clock, worker=worker)
        contexts = []
        decode = runtime.adapter.decode_action

        def capture(raw, context):
            contexts.append(context)
            return decode(raw, context)

        runtime.adapter.decode_action = capture
        result = runtime.run_policy(max_steps=4)
        self.assertGreater(result["accepted_plans"], 0)
        self.assertTrue(contexts)
        context = contexts[0]
        request_state = worker.submitted[0].observation.state
        self.assertEqual(context.ik_seed_source, "measured_state")
        self.assertEqual(context.observation_state.monotonic_ns, request_state.monotonic_ns)
        np.testing.assert_array_equal(context.observation_state.groups["left_arm"], request_state.groups["left_arm"])
        self.assertGreater(context.ik_seed.groups["left_arm"][0], context.observation_state.groups["left_arm"][0])

    def test_safe_hold_waits_for_fixed_period_task_termination(self):
        class HeldSim(FakeAsyncSim):
            def health(self):
                self.sim_ts += .04
                result = super().health()
                result["episode"]["state"] = "safe_hold" if self.sim_ts < .2 else "failed"
                return result

            def result(self):
                result = super().result()
                result["state"] = "failed"
                return result

        clock = FixedClock()
        backend = HeldSim()
        run = AsyncSimRuntime(config(), client=backend, clock=clock,
                              worker=FakeWorker(clock)).run_policy(max_steps=8)
        self.assertEqual(run["asyncsim_result"]["state"], "failed")
        self.assertFalse(backend.commands)

    def test_auxiliary_observations_keep_source_packet_identity(self):
        class ExtraSim(FakeAsyncSim):
            def read_snapshot(self, streams):
                snapshot = super().read_snapshot([name for name in streams if name != "proprio.control"])
                snapshot["packets"]["proprio.control"] = {"packet": {
                    "episode_id": self.episode_id, "seq": 9, "capture_ts": .02,
                    "available_ts": .03, "payload": {0: {"opening": [.2]}},
                }}
                return snapshot

        configured = config()
        configured["run"]["observation_streams"] = ["proprio.control"]
        backend = ExtraSim()
        # This fake accepts a subset read while synthesizing the auxiliary stream.
        backend.streams = ["proprio.joint_state", "camera.head.rgb"]
        backend.subscribe = lambda streams: None
        runtime = AsyncSimRuntime(configured, client=backend, clock=FixedClock())
        runtime.start()
        try:
            packet = runtime.observe().stream_packets["proprio.control"]
            self.assertEqual(packet["seq"], 9)
            self.assertEqual(packet["capture_ts"], .02)
            self.assertEqual(packet["available_ts"], .03)
            self.assertEqual(packet["payload"][0]["opening"], [.2])
        finally:
            runtime.close()

    def test_specialized_strategies_use_their_requests_and_canonical_timeline(self):
        for mode, request_type in (
            ("rtc", RtcInferenceRequest),
            ("paint", PaintInferenceRequest),
        ):
            with self.subTest(mode=mode):
                configured = config()
                configured["inference"]["algorithm"] = mode
                configured["inference"].update(inference_schedule="single_inflight", request_trigger="algorithm")
                configured["policy"]["adapter"]["type"] = "manimux.policy_adapter.joint:JointAdapter"
                configured["inference"]["rtc"].update(
                    initial_delay_policy_steps=1, min_execute_policy_steps=1,
                )
                configured["inference"]["paint"].update(
                    initial_delay_policy_steps=1, execution_policy_steps=1,
                )
                backend = FakeAsyncSim()
                clock = FixedClock()
                worker = SamplingWorker(clock, mode)
                result = AsyncSimRuntime(configured, client=backend, clock=clock, worker=worker).run_policy(max_steps=8)
                self.assertGreaterEqual(result["accepted_plans"], 1, result)
                if mode == "paint":
                    self.assertNotIsInstance(worker.submitted[0], PaintInferenceRequest)
                    self.assertTrue(any(isinstance(item, request_type) for item in worker.submitted[1:]))
                else:
                    self.assertIsInstance(worker.submitted[0], request_type)
                self.assertTrue(all(item["action_space"] == "joint_position" for item in backend.commands))

    def test_rtc_uses_simulation_time_for_conditioned_overlap(self):
        class SlowClock(FixedClock):
            def sleep_until_ns(self, target_ns):
                self.time_ns = max(self.time_ns, target_ns)

        class SlowAsyncSim(FakeAsyncSim):
            def __init__(self, clock):
                super().__init__(clock_ns=clock.now_ns)
                self.clock = clock

            def read_snapshot(self, streams):
                self.clock.time_ns += 200_000_000
                self.sim_ts += 0.04
                return super().read_snapshot(streams)

        clock = SlowClock()
        backend = SlowAsyncSim(clock)
        configured = config()
        configured["run"].update(simulation_time_timeline=True, instruction_stream="language.instruction")
        configured["robot"]["control_hz"] = 25
        configured["policy"]["action_dt_s"] = 0.04
        configured["policy"]["adapter"]["type"] = "manimux.policy_adapter.joint:JointAdapter"
        configured["inference"]["algorithm"] = "rtc"
        configured["inference"].update(inference_schedule="single_inflight", request_trigger="algorithm")
        configured["inference"]["rtc"].update(
            initial_delay_policy_steps=1, min_execute_policy_steps=1,
        )
        worker = SamplingWorker(clock, "rtc")
        result = AsyncSimRuntime(configured, client=backend, clock=clock, worker=worker).run_policy(max_steps=9)
        self.assertGreaterEqual(result["accepted_plans"], 2, result)
        self.assertTrue(any(request.condition_weights is not None for request in worker.submitted[1:]))
        self.assertTrue(all(request.instruction == backend.instruction for request in worker.submitted))
        self.assertTrue(all(request.deadline_ns > request.observation_time_ns for request in worker.submitted))
        self.assertTrue(all(command["action_space"] == "joint_position" for command in backend.commands))

    def test_sampling_mode_mismatch_fails_before_request(self):
        configured = config()
        configured["inference"]["algorithm"] = "rtc"
        configured["inference"].update(inference_schedule="single_inflight", request_trigger="algorithm")
        worker = FakeWorker(FixedClock())
        runtime = AsyncSimRuntime(configured, client=FakeAsyncSim(), clock=worker.clock, worker=worker)
        with self.assertRaisesRegex(RuntimeError, "sampling modes"):
            runtime.run_policy(max_steps=2)
        self.assertFalse(worker.submitted)

    def test_process_decoding_remains_explicitly_unsupported(self):
        configured = config()
        configured["policy"]["action_decoding"] = "process"
        with self.assertRaisesRegex(ValueError, "inline"):
            AsyncSimRuntime(configured, client=FakeAsyncSim())

    def test_act_aggregation_uses_manimux_strategy(self):
        backend = FakeAsyncSim()
        clock = FixedClock()
        configured = config()
        configured["inference"]["algorithm"] = "act_temporal_ensemble"
        configured["policy"]["adapter"]["type"] = "manimux.policy_adapter.joint:JointAdapter"
        runtime = AsyncSimRuntime(configured, client=backend, clock=clock, worker=FakeWorker(clock))
        result = runtime.run_policy(max_steps=4)
        self.assertEqual(runtime.strategy.name, "act_temporal_ensemble")
        self.assertGreaterEqual(result["accepted_plans"], 1, result)
        self.assertTrue(all(item["action_space"] == "joint_position" for item in backend.commands))

    def test_smooth_executor_filters_canonical_command(self):
        backend = FakeAsyncSim()
        configured = config()
        configured["executor"]["type"] = "smooth"
        runtime = AsyncSimRuntime(configured, client=backend, clock=FixedClock())
        runtime.start()
        runtime.step()
        chunk = ActionChunk("plan-smooth", 0, runtime.clock.now_ns(), runtime.clock.now_ns(),
                            "joint_position", 20_000_000, {
                                "left_arm": np.ones((3, 2)),
                                "right_arm": np.ones((3, 2)),
                            })
        _, result, _ = runtime.step(chunk)
        self.assertTrue(result.accepted)
        self.assertGreater(backend.commands[-1]["groups"]["left_arm"][0], 0.0)
        self.assertLess(backend.commands[-1]["groups"]["left_arm"][0], 1.0)
        runtime.close()

    def test_drop_infer_latency_reuses_one_chunk_for_multiple_commands(self):
        backend = FakeAsyncSim()
        clock = FixedClock()
        configured = config()
        configured["run"]["simulation_time_timeline"] = True
        configured["inference"]["action_start_mode"] = "drop_infer_latency"
        runtime = AsyncSimRuntime(configured, client=backend, clock=clock)
        runtime.start()
        try:
            runtime.observe()
            backend.sim_ts = 0.04
            observation = runtime.observe()
            groups = {name: np.repeat(np.arange(4)[:, None], 2, axis=1)
                      for name in ("left_arm", "right_arm")}
            chunk = ActionChunk(
                "plan-trimmed", 0, clock.now_ns(), clock.now_ns(),
                "joint_position", 20_000_000, groups,
            )
            result, _ = runtime.execute(observation, chunk)
            self.assertTrue(result.accepted)
            self.assertEqual(result.trimmed_steps, 2)
            self.assertEqual(backend.commands[-1]["groups"]["left_arm"], [2.0, 2.0])

            backend.sim_ts = 0.06
            observation = runtime.observe()
            runtime.execute(observation)
            self.assertEqual(backend.commands[-1]["groups"]["left_arm"], [3.0, 3.0])
            self.assertEqual([cmd["plan_id"] for cmd in backend.commands], ["plan-trimmed"] * 2)
        finally:
            runtime.close()

    def test_xpolicylab_template_loads_client_without_network(self):
        source = Path(__file__).resolve().parents[2] / (
            "manimux/configs/experiments/asyncsim/xpolicylab_joint_template.yaml"
        )
        configured = load_config(source)
        self.assertEqual(configured["robot"]["group_dims"], {"left_arm": 7, "right_arm": 7})
        self.assertEqual(configured["robot"]["options"]["state_stream"], "proprio.canonical_joint_state")
        backend = build_policy_model(configured["policy"])
        self.assertIsInstance(backend, XPolicyLabWsPolicyModel)
        backend.close()

    def test_robodojo_pi05_experiments_match_official_joint_contract(self):
        base = Path(__file__).resolve().parents[2] / "manimux/configs/experiments/asyncsim"
        for task, steps in (("insert_key", 300), ("match_and_pick_from_conveyor", 700)):
            with self.subTest(task=task):
                configured = load_config(base / f"robodojo_pi05_{task}.yaml")
                self.assertEqual(configured["run"]["task"], task)
                self.assertEqual(configured["run"]["instruction_stream"], "language.instruction")
                self.assertEqual(configured["run"]["max_control_steps"], steps)
                self.assertEqual(configured["robot"]["control_hz"], 25)
                self.assertEqual(configured["policy"]["action_dt_s"], 0.04)
                self.assertEqual(configured["policy"]["horizon_policy_steps"], 50)
                self.assertEqual(configured["inference"]["inference_schedule"], "single_inflight")
                self.assertEqual(configured["inference"]["action_start_mode"], "drop_infer_latency")
                self.assertEqual(configured["policy_server"]["checkpoint_num"], 59999)
                self.assertEqual(configured["policy_server"]["task_name"], task)
                self.assertEqual(configured["policy"]["expected_backend"]["model"]["task_name"], task)
                self.assertEqual(configured["policy"]["expected_backend"]["model"]["repo_id"], "arx_x5_sim")
                backend = build_policy_model(configured["policy"])
                self.assertIsInstance(backend, XPolicyLabWsPolicyModel)
                backend.close()

        for task, steps in (("insert_key", 300), ("match_and_pick_from_conveyor", 700)):
            with self.subTest(rtc_task=task):
                rtc = load_config(base / f"robodojo_pi05_{task}_rtc.yaml")
                self.assertEqual(rtc["run"]["task"], task)
                self.assertEqual(rtc["run"]["max_control_steps"], steps)
                self.assertEqual(rtc["policy_server"]["task_name"], task)
                self.assertEqual(rtc["robot"]["control_hz"], 25)
                self.assertEqual(rtc["policy"]["action_dt_s"], 0.04)
                self.assertEqual(rtc["policy"]["horizon_policy_steps"], 50)
                self.assertEqual(rtc["inference"]["algorithm"], "rtc")
                self.assertEqual(rtc["inference"]["rtc"]["min_execute_policy_steps"], 12)
                self.assertEqual(rtc["inference"]["rtc"]["initial_delay_policy_steps"], 4)
                self.assertIsNone(rtc["inference"]["max_chunk_policy_steps"])
                self.assertTrue(rtc["run"]["simulation_time_timeline"])

    def test_instruction_stream_and_inflight_action_replacement(self):
        backend = FakeAsyncSim()
        clock = FixedClock()
        configured = config()
        configured["run"]["instruction_stream"] = "language.instruction"
        configured["policy"]["adapter"]["type"] = "manimux.policy_adapter.joint:JointAdapter"
        configured["inference"].update(inference_schedule="single_inflight", refill_threshold_s=0.08)
        worker = DelayedWorker(clock)
        result = AsyncSimRuntime(configured, client=backend, clock=clock, worker=worker).run_policy(max_steps=12)
        events = result["audit"]["manimux_events"]
        self.assertIn("language.instruction", backend.streams)
        self.assertEqual(worker.submitted[0].instruction, backend.instruction)
        commits = [item for item in events if item["kind"] == "plan_committed"]
        self.assertGreaterEqual(len(commits), 2)
        first, second = commits[:2]
        submitted = next(index for index, item in enumerate(events)
                         if item["kind"] == "inference_submitted" and item["request_seq"] == second["request_seq"])
        second_commit = events.index(second)
        self.assertTrue(any(item["kind"] == "command_ack" and item["plan_id"] == first["plan_id"]
                            for item in events[submitted:second_commit]))
        self.assertTrue(any(item["kind"] == "command_ack" and item["plan_id"] == second["plan_id"]
                            for item in events[second_commit:]))
        self.assertIn("issued_sim_ts", result["command_acks"][0])

    def test_missing_instruction_does_not_fall_back_to_task_id(self):
        configured = config()
        configured["run"]["instruction_stream"] = "language.instruction"
        backend = FakeAsyncSim()
        backend.instruction = ""
        with self.assertRaisesRegex(ValueError, "non-empty string"):
            AsyncSimRuntime(configured, client=backend, clock=FixedClock()).run_policy(max_steps=2)

    def test_model_wait_does_not_consume_robodojo_action_limit(self):
        configured = config()
        configured["run"]["skip_unplanned_commands"] = True
        configured["policy"]["adapter"]["type"] = "manimux.policy_adapter.joint:JointAdapter"
        configured["inference"].update(inference_schedule="single_inflight", refill_threshold_s=0.08)
        backend = FakeAsyncSim()
        clock = FixedClock()
        result = AsyncSimRuntime(configured, client=backend, clock=clock, worker=DelayedWorker(clock)).run_policy(max_steps=3)
        self.assertEqual(result["steps"], 3)
        self.assertGreater(result["loop_ticks"], result["steps"])
        self.assertEqual(len(backend.commands), 3)
        self.assertTrue(all(command["plan_id"].startswith("joint-") for command in backend.commands))
        self.assertEqual(result["asyncsim_result"]["result"]["action_count"], 3)

    def test_stale_policy_response_is_rejected(self):
        backend = FakeAsyncSim()
        clock = FixedClock()
        worker = StaleWorker(clock)
        runtime = AsyncSimRuntime(config(), client=backend, clock=clock, worker=worker)
        result = runtime.run_policy(max_steps=3)
        self.assertEqual(result["accepted_plans"], 0)
        self.assertIn("stale_session", result["rejection_reasons"])

    def test_policy_worker_adapter_and_timeline_pipeline(self):
        backend = FakeAsyncSim()
        clock = FixedClock()
        worker = FakeWorker(clock)
        configured = config()
        configured["policy"]["adapter"]["type"] = "manimux.policy_adapter.joint:JointAdapter"
        runtime = AsyncSimRuntime(configured, client=backend, clock=clock, worker=worker)
        result = runtime.run_policy(max_steps=3)
        self.assertGreaterEqual(result["accepted_plans"], 1, result)
        self.assertTrue(worker.submitted)
        self.assertEqual(worker.submitted[0].observation.frames["head"].sequence, 1)
        self.assertTrue(any(command["plan_id"].startswith("joint-") for command in backend.commands))
        self.assertEqual(len(backend.commands), 3)
        self.assertEqual(len(result["command_acks"]), 3)
        self.assertEqual(result["asyncsim_result"]["result"]["action_count"], 3)
        self.assertEqual(result["asyncsim_metrics"]["episode_id"], result["episode_id"])
        self.assertTrue(any(item["kind"] == "plan_committed" for item in result["audit"]["manimux_events"]))
        self.assertEqual(len(result["audit"]["actions"]), 3)
        self.assertEqual(len(result["packets"]), 2)
        self.assertTrue(backend.closed)

    def test_real_fake_policy_worker_process(self):
        backend = FakeAsyncSim(time.monotonic_ns)
        configured = config()
        configured["policy"]["timeout_s"] = 3.0
        configured["inference"]["max_plan_age_s"] = 3.0
        runtime = AsyncSimRuntime(configured, client=backend)
        result = runtime.run_policy(max_steps=20)
        self.assertGreaterEqual(result["accepted_plans"], 1, result)
        self.assertTrue(any(item["plan_id"].startswith("plan-") for item in backend.commands))

    def test_policy_backend_identity_mismatch_aborts(self):
        backend = FakeAsyncSim()
        clock = FixedClock()
        configured = config()
        configured["policy"]["expected_backend"] = {"server": "expected"}
        worker = FakeWorker(clock, PolicyCapabilities(backend_metadata={"server": "other"}))
        runtime = AsyncSimRuntime(configured, client=backend, clock=clock, worker=worker)
        with self.assertRaisesRegex(RuntimeError, "identity mismatch"):
            runtime.run_policy(max_steps=2)
        self.assertFalse(worker.submitted)
        self.assertTrue(backend.closed)

    def test_pi05_station_resolves_read_only_robodojo_checkpoint(self):
        base = Path(__file__).resolve().parents[2]
        source = base / "manimux/configs/experiments/asyncsim/robodojo_pi05_insert_key.yaml"
        local = base / "manimux/configs/local/asyncsim_pi05.example.yaml"
        configured = load_config(source, local=local)
        expected = configured["policy"]["expected_backend"]["model"]
        self.assertEqual(expected["model_root"], configured["policy_server"]["model_path"])
        self.assertEqual(expected["norm_stats_path"], configured["policy_server"]["norm_stats_path"])
        self.assertTrue(Path(expected["model_root"]).joinpath("params").is_dir())
        self.assertTrue(Path(expected["norm_stats_path"]).joinpath("norm_stats.json").is_file())
        from manimux.servers.asyncsim_pi05 import resolve_server

        self.assertEqual(resolve_server(source, local)["model_path"], expected["model_root"])

    def test_wrong_pi05_checkpoint_aborts_before_command(self):
        source = Path(__file__).resolve().parents[2] / (
            "manimux/configs/experiments/asyncsim/robodojo_pi05_insert_key.yaml"
        )
        local = source.parents[2] / "local/asyncsim_pi05.example.yaml"
        configured = load_config(source, local=local)
        clock = FixedClock()
        expected = configured["policy"]["expected_backend"]
        reported = {"server": expected["server"], "model": {
            **expected["model"], "model_root": "/another-checkpoint/59999",
        }}
        worker = FakeWorker(clock, PolicyCapabilities(backend_metadata=reported))
        backend = FakeAsyncSim()
        backend.health = lambda: {
            "episode": {"sim_ts": backend.sim_ts, "state": "running"},
            "canonical_group_dims": {"left_arm": 7, "right_arm": 7},
        }
        with self.assertRaisesRegex(RuntimeError, "model_root"):
            AsyncSimRuntime(configured, client=backend, clock=clock, worker=worker).run_policy(max_steps=2)
        self.assertFalse(worker.submitted)
        self.assertFalse(backend.commands)

    def test_fake_backend_uses_one_snapshot_and_one_command_per_tick(self):
        backend = FakeAsyncSim()
        runtime = AsyncSimRuntime(config(), client=backend, clock=FixedClock())
        runtime.start(seed=3)
        observation, result, ack = runtime.step()
        self.assertIsNone(result)
        self.assertTrue(ack["accepted"])
        self.assertEqual(observation.state.sequence, 3)
        self.assertEqual(observation.frames["head"].data.shape, (2, 3, 3))
        self.assertEqual(backend.commands[0]["plan_id"], "hold")
        self.assertEqual(backend.streams, ["proprio.joint_state", "camera.head.rgb"])
        chunk = ActionChunk("plan-1", 0, runtime.clock.now_ns(), runtime.clock.now_ns(),
                            "joint_position", 20_000_000, {
                                "left_arm": np.ones((3, 2)),
                                "right_arm": np.ones((3, 2)),
                            })
        _, result, _ = runtime.step(chunk)
        self.assertTrue(result.accepted)
        self.assertEqual(backend.commands[-1]["groups"]["left_arm"], [1.0, 1.0])
        self.assertEqual([item["command_seq"] for item in backend.commands], [1, 2])
        runtime.close()
        self.assertTrue(backend.closed)
        runtime.start(seed=3)
        runtime.step()
        self.assertEqual(backend.commands[-1]["command_seq"], 1)
        runtime.close()

    def test_send_command_reuses_snapshot_time_without_health(self):
        class NoHealth(FakeAsyncSim):
            def health(self):
                raise AssertionError("sending a command must not request health")

        backend = NoHealth()
        backend.reset()
        backend.subscribe(["proprio.joint_state"])
        backend.sim_ts = .04
        snapshot = backend.read_snapshot(backend.streams)
        snapshot["packets"]["proprio.joint_state"]["packet"]["capture_ts"] = .02
        robot = AsyncSimRobot(backend, {"left_arm": 2}, {"left_arm": "left_joints"})
        robot.use_snapshot(snapshot)
        backend.sim_ts = .084
        ack = robot.send_command(RobotCommand({"left_arm": np.ones(2)}, 1_000_000_000, "plan-1"))
        issued = backend.commands[0]
        self.assertEqual(issued["issued_sim_ts"], .04)
        self.assertEqual(issued["apply_after_sim_ts"], .04)
        self.assertAlmostEqual(issued["expires_at_sim_ts"], .24)
        self.assertEqual(ack["issued_sim_ts"], .04)
        self.assertEqual(issued["command_seq"], 1)

    def test_sensor_rejects_non_uint8_and_robot_rejects_wrong_layout(self):
        backend = FakeAsyncSim()
        backend.reset()
        backend.subscribe(["proprio.joint_state", "camera.head.rgb"])
        snapshot = backend.read_snapshot(["proprio.joint_state", "camera.head.rgb"])
        sensor = AsyncSimSensor(backend, {"head": "camera.head.rgb"})
        sensor.start()
        snapshot["packets"]["camera.head.rgb"]["packet"]["payload"][0] = np.ones((2, 2, 3))
        with self.assertRaises(ValueError):
            sensor.use_snapshot(snapshot)
        robot = AsyncSimRobot(backend, {"left_arm": 2}, {"left_arm": "left_joints"})
        robot.use_snapshot(snapshot)
        with self.assertRaises(ValueError):
            robot.send_command(RobotCommand({"left_arm": [1.0]}, 1_000_000_000, "bad"))


if __name__ == "__main__":
    unittest.main()
