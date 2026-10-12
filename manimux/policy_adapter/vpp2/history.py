"""Native-period history decorating the existing serial inference strategy."""

from copy import deepcopy
from dataclasses import replace

from manimux.policies.base import action_interval
from manimux.policy_adapter.vpp2.arx_x5 import REPLAN_STEPS, VPP2ArxX5Adapter
from manimux.runtime.inference import build_inference_strategy
from manimux.runtime.safety import RuntimeState


class HistoryStrategy:
    """Use visible control-tick observations and the controller's local timeline.

    No simulator APIs or actuator application assertions are involved. Gripper
    conditioning uses nominal committed targets, not dispatch acknowledgements.
    """

    discard_plans_while_paused = True

    def __init__(self, config):
        inference = config["inference"]
        if (
            inference["algorithm"] != "serial"
            or inference["inference_schedule"] != "serial"
            or inference["action_start_mode"] != "first_step_when_ready"
            or inference["max_chunk_policy_steps"] != REPLAN_STEPS
            or inference["blend_policy_steps"] != 0
            or inference["handoff_skip_steps"] != 0
            or config["executor"]["type"] != "direct"
            or config["executor"].get("motion_limits") is not None
            or config["policy"]["action_decoding"] != "inline"
            or config["robot"]["control_hz"] != 25
            or action_interval(config["policy"]) != .04
            or config["policy"]["worker"] != "manimux.policies.xpolicylab.vpp2:build_model"
        ):
            raise ValueError("VPP2 history requires serial native 24-step direct execution and its CALL wrapper")
        delegate = deepcopy(config)
        delegate["inference"]["strategy"] = None
        self.delegate = build_inference_strategy(delegate)
        self.dt_ns = round(action_interval(config["policy"]) * 1e9)
        self._session_id = None
        self.reset()

    def __getattr__(self, name):
        return getattr(self.delegate, name)

    def reset(self):
        # Edge may reset scheduling on Pause/Home without resetting the backend.
        # Such a reset must not append a second anchor to an existing model history.
        self._requires_new_session = self._session_id is not None
        self._completed = []
        self._pending_request = None
        self._plan_id = None
        self._has_anchor = False
        self._adapter = None
        self._timeline = None
        self.delegate.reset()

    def observe_snapshot(self, snapshot):
        if not self._has_anchor or self._pending_request is not None or len(self._completed) == REPLAN_STEPS:
            return
        active = self._timeline.active_horizon()
        if active is None or active.plan_id != self._plan_id:
            raise RuntimeError("VPP2 history lost its committed plan")
        if active.dt_ns != self.dt_ns or active.horizon_steps != REPLAN_STEPS:
            raise RuntimeError("VPP2 history needs an untrimmed native 24-step plan")
        row = len(self._completed)
        boundary_ns = active.start_time_ns + (row + 1) * self.dt_ns
        # Use the measured state's capture clock, so stale feedback cannot fill a later period.
        sample_ns = snapshot.state.monotonic_ns
        if sample_ns < boundary_ns:
            return
        if sample_ns >= boundary_ns + self.dt_ns:
            raise RuntimeError("VPP2 missed a native observation boundary; history cannot be backfilled")
        targets = {name: values[row, -1] for name, values in active.groups.items()}
        self._completed.append(self._adapter.build_observation(snapshot, gripper_targets=targets))

    def build_submission(self, **kwargs):
        if kwargs["runtime_state"] != RuntimeState.RUNNING:
            return None
        session_id = kwargs["session_id"]
        if self._requires_new_session:
            if session_id == self._session_id:
                raise RuntimeError("VPP2 history reset requires a new backend session")
            self._requires_new_session = False
            self._session_id = None
        if self._session_id is not None and session_id != self._session_id:
            raise RuntimeError("reset VPP2 history before switching sessions")
        self._session_id = session_id
        if self._pending_request is not None:
            return None
        adapter = kwargs["adapter"]
        if not isinstance(adapter, VPP2ArxX5Adapter):
            raise TypeError("VPP2 history requires its paired observation/action adapter")
        self._adapter, self._timeline = adapter, kwargs["timeline"]
        snapshot = kwargs["snapshot"]
        targets = None
        if self._has_anchor:
            active = kwargs["timeline"].active_horizon()
            if active is None or active.plan_id != self._plan_id:
                raise RuntimeError("VPP2 history lost its committed plan")
            if active.dt_ns != self.dt_ns or active.horizon_steps != REPLAN_STEPS:
                raise RuntimeError("VPP2 history needs an untrimmed native 24-step plan")
            if len(self._completed) < REPLAN_STEPS:
                return None
            row = REPLAN_STEPS - 1
            targets = {name: values[row, -1] for name, values in active.groups.items()}
        view = adapter.build_observation(snapshot, gripper_targets=targets)
        history = (*self._completed[:-1], view) if self._has_anchor else (view,)
        window = replace(view, observation_history=history)
        submission = self.delegate.build_submission(**{**kwargs, "snapshot": window})
        if submission is not None:
            self._pending_request = submission.request.request_seq
        return submission

    def on_plan_accepted(self, *, chunk, result, response, now_ns):
        if response.session_id != self._session_id or response.request_seq != self._pending_request:
            raise RuntimeError("VPP2 stateful plan does not match its pending request")
        if result.trimmed_steps or chunk.horizon_steps != REPLAN_STEPS or chunk.dt_ns != self.dt_ns:
            raise RuntimeError("VPP2 accepted plan must preserve its native 24-step prefix")
        self._completed.clear()
        self._pending_request = None
        self._has_anchor = True
        self._plan_id = chunk.plan_id
        return self.delegate.on_plan_accepted(chunk=chunk, result=result, response=response, now_ns=now_ns)

    def on_response_rejected(self, response):
        raise RuntimeError(f"VPP2 stateful request cannot be retried without a new backend session: {response.error or 'response rejected'}")
