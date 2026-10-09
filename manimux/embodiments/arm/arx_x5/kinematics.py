"""RoboDojo X5 link6 FK/IK in base_link coordinates, without a simulator.

The six revolute joints follow the RoboDojo URDF by name. The seventh public
coordinate is its normalized gripper command, independent of link6 geometry.
Mink solves the six arm coordinates with URDF position limits. This is a local
pose solver, not cuRobo's collision-aware planner or a hardware driver.
"""

from __future__ import annotations

import threading
import xml.etree.ElementTree as ET
from collections.abc import Mapping
from pathlib import Path

import mujoco
import numpy as np

from manimux.kinematics.base import (
    IKResult,
    KinematicCoordinate,
    ManipulatorKinematicsBase,
    rigid_transform,
)

ARM_JOINTS = tuple(f"joint{i}" for i in range(1, 7))
DEFAULT_URDF = Path(__file__).with_name("description.urdf")


def _arm_xml(path: Path) -> str:
    """Keep only base_link -> link6; fingers are fixed outside the IK problem."""
    root = ET.parse(path).getroot()
    links = {"base_link", *(f"link{i}" for i in range(1, 7))}
    joints = {item.get("name"): item for item in root.findall("joint")}
    for index, name in enumerate(ARM_JOINTS):
        joint = joints.get(name)
        parent = "base_link" if index == 0 else f"link{index}"
        if (
            joint is None or joint.get("type") != "revolute"
            or joint.find("parent").get("link") != parent
            or joint.find("child").get("link") != f"link{index + 1}"
        ):
            raise ValueError(f"X5 URDF must contain the ordered base_link/link6 chain: {name}")
    for item in list(root):
        if item.tag == "joint" and item.get("name") not in ARM_JOINTS:
            root.remove(item)
        elif item.tag == "link":
            if item.get("name") not in links:
                root.remove(item)
            else:
                for geometry in list(item):
                    if geometry.tag in {"visual", "collision"}:
                        item.remove(geometry)
        elif item.tag not in {"link", "joint"}:
            root.remove(item)
    return ET.tostring(root, encoding="unicode")


class ArxX5Kinematics(ManipulatorKinematicsBase):
    """Six radians + gripper in [0, 1]; TCP is link6 with no grasp offset.

    IK supports exactly {"gripper": value} as fixed coordinates. Failure never
    returns candidate joints. Every accepted solution is checked after clipping
    roundoff at the joint stops. Calls are serialized around mutable MuJoCo data.
    """

    coordinates = tuple(KinematicCoordinate(name, "rad") for name in ARM_JOINTS) + (
        KinematicCoordinate("gripper", "normalized"),
    )
    base_frame = "base_link"
    tcp_frame = "link6"

    def __init__(
        self, *, urdf_path: str | Path = DEFAULT_URDF,
        position_tolerance: float = 1e-5, orientation_tolerance: float = 1e-4,
        max_iterations: int = 200,
    ):
        tolerances = np.array([position_tolerance, orientation_tolerance], dtype=float)
        if not np.isfinite(tolerances).all() or np.any(tolerances <= 0):
            raise ValueError("pose tolerances must be finite and positive")
        if (
            isinstance(max_iterations, bool)
            or not isinstance(max_iterations, int) or max_iterations < 1
        ):
            raise ValueError("max_iterations must be a positive integer")
        self.position_tolerance, self.orientation_tolerance = tolerances
        self.max_iterations = max_iterations
        self.urdf_path = Path(urdf_path).expanduser().resolve()
        self.model = mujoco.MjModel.from_xml_string(_arm_xml(self.urdf_path))
        self._data = mujoco.MjData(self.model)
        joint_ids = [self.model.joint(name).id for name in ARM_JOINTS]
        self._addresses = self.model.jnt_qposadr[joint_ids]
        if self.model.nq != 6 or not np.array_equal(self._addresses, np.arange(6)):
            raise ValueError("X5 solver model must have exactly joint1 through joint6 in order")
        if not self.model.jnt_limited[joint_ids].all():
            raise ValueError("X5 arm joints must declare finite position limits")
        self._lower, self._upper = self.model.jnt_range[joint_ids].T.copy()
        self._body_id = self.model.body(self.tcp_frame).id
        self._lock = threading.Lock()
        self._configuration = None

    @staticmethod
    def _configuration_vector(value):
        q = np.array(value, dtype=float, copy=True)
        if q.shape != (7,) or not np.isfinite(q).all():
            raise ValueError("X5 configuration must contain seven finite coordinates")
        if not 0 <= q[6] <= 1:
            raise ValueError("X5 gripper must be normalized to [0, 1]")
        return q

    def joint_position_limits(self):
        return self._lower.copy(), self._upper.copy()

    def visual_configuration(self, configuration):
        """Expand the RoboDojo command mapping, including its closing overtravel.

        URDF/USD finger limits are [0, .044] m, whereas RoboDojo commands
        -.01 + .054 * opening. Physics may clamp the actual finger position.
        This method maps commanded coordinates; it does not invent feedback.
        """
        q = self._configuration_vector(configuration)
        finger = -0.01 + 0.054 * q[6]
        return np.r_[q[:6], finger, finger]

    def _fk(self, joints):
        self._data.qpos[self._addresses] = joints
        mujoco.mj_kinematics(self.model, self._data)
        result = np.eye(4)
        result[:3, :3] = self._data.xmat[self._body_id].reshape(3, 3)
        result[:3, 3] = self._data.xpos[self._body_id]
        return result

    def fk(self, configuration):
        q = self._configuration_vector(configuration)
        with self._lock:
            return self._fk(q[:6])

    def _errors(self, target, joints):
        actual = self._fk(joints)
        position = np.linalg.norm(target[:3, 3] - actual[:3, 3])
        cosine = (np.trace(actual[:3, :3].T @ target[:3, :3]) - 1) / 2
        rotation = np.arccos(np.clip(cosine, -1, 1))
        return position, rotation

    def ik(self, target_tcp, seed_configuration, *, fixed_coordinates: Mapping[str, float]):
        target = rigid_transform(target_tcp, "target_tcp")
        seed = self._configuration_vector(seed_configuration)
        unknown = set(fixed_coordinates) - {coordinate.name for coordinate in self.coordinates}
        if unknown:
            raise ValueError(f"unknown fixed coordinates: {sorted(unknown)}")
        if set(fixed_coordinates) != {"gripper"}:
            raise NotImplementedError(
                "X5 IK requires the gripper fixed and all six arm joints free"
            )
        grip = np.asarray(fixed_coordinates["gripper"], dtype=float)
        if grip.shape != () or not np.isfinite(grip) or not 0 <= grip <= 1:
            raise ValueError("fixed gripper must be a finite scalar in [0, 1]")
        if np.any(seed[:6] < self._lower) or np.any(seed[:6] > self._upper):
            return IKResult(False, reason="seed_outside_joint_limits")

        import mink

        with self._lock:
            if self._configuration is None:
                self._configuration = mink.Configuration(self.model)
                self._limits = [mink.ConfigurationLimit(self.model)]
            configuration = self._configuration
            configuration.update(seed[:6])
            task = mink.FrameTask(
                self.tcp_frame, "body", position_cost=1.0,
                orientation_cost=1.0, lm_damping=1.0,
            )
            task.set_target(mink.SE3.from_matrix(target))
            best, stale = float("inf"), 0
            reason = "iteration_limit"
            for iteration in range(self.max_iterations + 1):
                joints = np.clip(configuration.q, self._lower, self._upper)
                position, rotation = self._errors(target, joints)
                if position <= self.position_tolerance and rotation <= self.orientation_tolerance:
                    return IKResult(True, joints=np.r_[joints, float(grip)])
                if iteration == self.max_iterations:
                    break
                score = max(
                    position / self.position_tolerance, rotation / self.orientation_tolerance
                )
                if score < best * (1 - 1e-5):
                    best, stale = score, 0
                else:
                    stale += 1
                if stale >= 15:
                    reason = "stagnation"
                    break
                velocity = mink.solve_ik(
                    configuration, [task], .01, "quadprog", damping=1e-5,
                    limits=self._limits,
                )
                configuration.integrate_inplace(velocity, .01)
            return IKResult(False, reason=reason)
