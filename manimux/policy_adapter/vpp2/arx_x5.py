"""VPP2 visible-state mapping and calibrated absolute X5 pose conversion."""

from __future__ import annotations

from dataclasses import dataclass, field
import uuid

import numpy as np

from manimux.kinematics.poses import matrix_pose, pose_matrix
from manimux.policies.actions import action_groups
from manimux.policies.base import action_interval
from manimux.policy_adapter.kinematics import KinematicAdapter
from manimux.types import ActionChunk, InferenceRequest, ObservationSnapshot, RobotState, copy_group_vector

GROUPS = ("left_arm", "right_arm")
REPLAN_STEPS = 24


@dataclass(slots=True)
class VPP2Snapshot(ObservationSnapshot):
    model_state: dict[str, np.ndarray] = field(default_factory=dict)
    observation_history: tuple[VPP2Snapshot, ...] = ()


@dataclass(slots=True)
class VPP2Request(InferenceRequest):
    observation_history: tuple[VPP2Snapshot, ...] = ()


class VPP2ArxX5Adapter(KinematicAdapter):
    """Map measured joints/cameras; scheduling stays in the history strategy."""

    def __init__(self, robot, policy, *, kinematics=None):
        super().__init__(robot, policy, kinematics=kinematics)
        self.validate(robot, policy)
        spec = policy["adapter"]
        self._cameras = tuple(spec["camera_map"].values())
        self._bases = {name: pose_matrix(spec["base_poses"][name]) for name in GROUPS}
        self._base_inverses = {name: np.linalg.inv(base) for name, base in self._bases.items()}
        self._dt_ns = round(action_interval(policy) * 1e9)

    def validate(self, robot, policy):
        if robot["group_dims"] != dict.fromkeys(GROUPS, 7):
            raise ValueError("VPP2 requires the X5 six-joint/scalar-gripper layout")
        if robot["control_hz"] != 25 or not np.isclose(action_interval(policy), .04):
            raise ValueError("VPP2 requires 25 Hz control and 40 ms native actions")
        if policy["horizon_policy_steps"] != REPLAN_STEPS or policy["options"].get("action_format") != "pose":
            raise ValueError("VPP2 requires a 24-row pose prefix per response")
        spec = policy["adapter"]
        if spec.get("action_semantics") != "absolute_env_relative_xyz_wxyz":
            raise ValueError("VPP2 requires absolute calibrated workspace link6 poses")
        if spec.get("gripper_observation") != "nominal_plan_target":
            raise ValueError("VPP2 requires explicit nominal_plan_target gripper semantics")
        if set(spec.get("camera_map", {})) != {"cam_head", "cam_left_wrist", "cam_right_wrist"}:
            raise ValueError("VPP2 requires all three RGB cameras")
        if set(spec.get("base_poses", {})) != set(GROUPS) or set(self.kinematics.models) != set(GROUPS):
            raise ValueError("VPP2 requires both calibrated arm bases and models")
        if any((model.base_frame, model.tcp_frame) != ("base_link", "link6") for model in self.kinematics.models.values()):
            raise ValueError("VPP2 requires base_link -> link6 kinematics")
        identity = (policy.get("expected_backend") or {}).get("model", {})
        if (identity.get("policy_name"), identity.get("action_type"), identity.get("action_horizon")) != ("VPP2", "ee", 32):
            raise ValueError("VPP2 requires a pinned EE32/execute24 backend identity")

    def build_observation(self, snapshot, *, gripper_targets=None):
        if isinstance(snapshot, VPP2Snapshot):
            return snapshot
        if any(name not in snapshot.frames for name in self._cameras):
            raise ValueError("VPP2 observation lacks a configured camera")
        groups = copy_group_vector(snapshot.state.groups)
        model_state = {}
        for name in GROUPS:
            prefix = name.removesuffix("_arm")
            model_state[f"{prefix}_ee_pose"] = matrix_pose(
                self._bases[name] @ self.kinematics.models[name].fk(groups[name])
            )
            opening = groups[name][-1] if gripper_targets is None else gripper_targets[name]
            model_state[f"{prefix}_ee_joint_state"] = np.array([np.clip(opening, 0, 1)])
        return VPP2Snapshot(
            RobotState(groups, snapshot.state.monotonic_ns, snapshot.state.sequence),
            {name: snapshot.frames[name] for name in self._cameras}, model_state=model_state,
        )

    def prepare_request(self, request):
        if not isinstance(request.observation, VPP2Snapshot):
            raise TypeError("VPP2 request needs its mapped sensor observation")
        history = request.observation.observation_history
        if len(history) not in {1, REPLAN_STEPS}:
            raise ValueError("VPP2 requires history from its configured inference.strategy")
        return VPP2Request(
            request.session_id, request.request_seq, request.observation_time_ns,
            request.deadline_ns, request.observation, request.instruction, history,
        )

    def decode_action(self, raw, context):
        if context.measured_state is None:
            raise ValueError("VPP2 action needs measured joint feedback as its IK seed")
        actions = action_groups(raw, dict.fromkeys(GROUPS, 8), format="pose")
        if any(len(rows) != REPLAN_STEPS for rows in actions.values()):
            raise ValueError("VPP2 must return exactly 24 executable rows")
        groups = {}
        for name in GROUPS:
            model = self.kinematics.models[name]
            seed = context.measured_state.groups[name].copy()
            seed[-1] = np.clip(seed[-1], 0, 1)
            rows = []
            for index, row in enumerate(actions[name]):
                target = self._base_inverses[name] @ pose_matrix(row[:7])
                result = model.ik(target, seed, fixed_coordinates={"gripper": float(np.clip(row[7], 0, 1))})
                if not result.converged:
                    raise ValueError(f"VPP2 IK rejected {name} row {index}: {result.reason}")
                seed = result.joints.copy()
                rows.append(seed)
            groups[name] = np.ascontiguousarray(rows)
        return ActionChunk(
            f"vpp2-x5-{context.request_seq}-{uuid.uuid4().hex[:8]}", context.request_seq, context.observation_time_ns,
            context.created_time_ns, "joint_position", self._dt_ns, groups,
        )
