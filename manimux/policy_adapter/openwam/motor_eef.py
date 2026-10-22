"""OpenWAM EEF source actions with runtime-owned physical motor conversion."""

from dataclasses import replace
from collections.abc import Mapping
import uuid

import numpy as np

from manimux.kinematics.poses import pose_matrix
from manimux.policies.actions import action_groups
from manimux.policies.base import action_interval
from manimux.policy_adapter.base import PolicyAdapter
from manimux.policy_adapter.openwam.arx_x5 import GROUPS, CAMERAS, SEMANTICS, OpenWAMX5Request
from manimux.types import ActionChunk, RobotState


class OpenWAMMotorEEFAdapter(PolicyAdapter):
    def __init__(self, robot, policy, *, kinematics=None):
        if kinematics is not None:
            raise ValueError("motor EEF adapter must not construct chunk-level kinematics")
        super().__init__(robot, policy)
        self.validate(robot, policy)
        self._env_index = robot["options"].get("env_index", 0)

    def validate(self, robot, policy):
        spec = policy["adapter"]
        identity = (policy.get("expected_backend") or {}).get("model", {})
        if (robot["type"] != "asyncsim" or robot["group_dims"] != dict.fromkeys(GROUPS, 8)
                or robot.get("config") is not None):
            raise ValueError("motor EEF requires an explicit eight-coordinate AsyncSim interface")
        if (spec.get("action_semantics") != SEMANTICS or set(spec.get("camera_map", {})) != CAMERAS
                or policy["options"].get("action_format") != "pose"
                or identity.get("policy_name") != "OpenWAM" or identity.get("action_type") != "ee"
                or identity.get("action_horizon") != policy["horizon_policy_steps"]):
            raise ValueError("motor EEF requires the pinned OpenWAM absolute-pose backend contract")

    def create_command_controller(self, config):
        from manimux.runtime.motor_control import RoboDojoMotorController

        return RoboDojoMotorController(config)

    def _state_packet(self, snapshot, stream):
        packet = snapshot.stream_packets.get(stream)
        if packet is None:
            raise ValueError(f"motor EEF observation lacks {stream}")
        payload = packet["payload"]
        if not isinstance(payload, Mapping):
            raise ValueError("motor EEF observation must be an environment mapping")
        state = payload.get(self._env_index, payload.get(str(self._env_index)))
        if not isinstance(state, Mapping):
            raise ValueError("motor EEF observation lacks the selected environment")
        return state

    def build_observation(self, snapshot):
        spec = self.policy["adapter"]
        if any(name not in snapshot.frames for name in spec["camera_map"].values()):
            raise ValueError("motor EEF observation lacks a configured camera")
        joints = self._state_packet(snapshot, spec["gripper_state_stream"])
        poses = self._state_packet(snapshot, spec["pose_state_stream"])
        groups = {}
        for name in GROUPS:
            prefix = name.removesuffix("_arm")
            arm = np.asarray(joints[f"{prefix}_arm_joint_state"], dtype=float)
            gripper = np.asarray(joints[f"{prefix}_ee_joint_state"], dtype=float)
            pose_matrix(poses[f"{prefix}_ee_pose"])
            if (arm.shape != (6,) or gripper.shape != (1,) or not np.isfinite(arm).all()
                    or not np.isfinite(gripper).all() or not -1e-8 <= gripper[0] <= 1 + 1e-8):
                raise ValueError("model observation requires six measured joints and one control opening")
            groups[name] = np.r_[arm, np.clip(gripper, 0, 1)]
        return replace(snapshot, state=RobotState(groups, snapshot.state.monotonic_ns, snapshot.state.sequence))

    def prepare_request(self, request):
        poses = self._state_packet(request.observation, self.policy["adapter"]["pose_state_stream"])
        state = {f"{name.removesuffix('_arm')}_ee_pose": np.array(
            poses[f"{name.removesuffix('_arm')}_ee_pose"], dtype=float, copy=True) for name in GROUPS}
        return OpenWAMX5Request(
            session_id=request.session_id, request_seq=request.request_seq,
            observation_time_ns=request.observation_time_ns, deadline_ns=request.deadline_ns,
            observation=request.observation, instruction=request.instruction, model_state=state,
        )

    def decode_action(self, raw, context):
        groups = action_groups(raw, dict.fromkeys(GROUPS, 8), format="pose")
        if any(len(rows) != self.policy["horizon_policy_steps"] for rows in groups.values()):
            raise ValueError("motor EEF action horizon mismatch")
        for rows in groups.values():
            for row in rows:
                pose_matrix(row[:7])
        return ActionChunk(
            plan_id=f"openwam-motor-eef-{context.request_seq}-{uuid.uuid4().hex[:8]}",
            request_seq=context.request_seq, observation_time_ns=context.observation_time_ns,
            created_time_ns=context.created_time_ns, action_space="eef_pose",
            dt_ns=round(action_interval(self.policy) * 1e9), groups=groups,
            metadata={"native_action_semantics": SEMANTICS, "controller_location": "manimux_runtime"},
        )
