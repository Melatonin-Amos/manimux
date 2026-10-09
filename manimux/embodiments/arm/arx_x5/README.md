# ARX X5 offline kinematics

This component supplies a `ManipulatorKinematicsBase` and an `ArmModel` for
RoboDojo's simulated X5. Load the two-arm assembly with
`RobotModel.from_config("manimux/configs/embodiment/robot/arx_x5_dual.yaml")`
from the ManiMux root. `ArxX5Arm` is an abstract, model-only provider: it has no
physical controller. AsyncSim continues to own simulation and joint execution.

Install the solver in a Python 3.11/3.12 environment with
`uv sync --extra arx-x5-ik`. FK uses MuJoCo; IK lazily imports Mink and Quadprog.
No Isaac Sim, torch, CUDA, camera or robot connection is required for the model.

## Coordinates and frames

Each group contains `[joint1, joint2, joint3, joint4, joint5, joint6, gripper]`.
Arm values are radians. Gripper is RoboDojo's normalized command in `[0, 1]`.
The geometry is `T_base_link_link6`, in metres. Left and right groups each use
their own arm base, independently of scene placement. A world/environment EEF
observation must be converted with that arm's actual base pose before comparison
or IK. Assembly/viewer placement must not be added a second time.

RoboDojo `get_real_endpose()` observes `link6`. The `gripper_bias = 0.145` used
elsewhere for grasp planning is **not** a TCP translation in this model. XR1's
MiBot axis reframe belongs to its policy adapter, not to this arm geometry.

The gripper command expands to two finger targets using
`finger = -0.01 + 0.054 * gripper`. This preserves RoboDojo's closing overtravel;
the URDF/USD physical finger limits are `[0, 0.044]` m. Command, measured finger
position, and normalized measurement are therefore distinct. Finger opening does
not change `link6` FK. The bundled description has no visual/collision meshes;
`visual_configuration()` only expands the command coordinates.

## IK contract

`ik(target_tcp, seed_configuration, fixed_coordinates={"gripper": opening})`
fixes the gripper and solves the six arm joints. Other fixed/free patterns raise
`NotImplementedError`; malformed inputs and unknown names raise `ValueError`.
FK accepts finite measured arm angles. IK rejects a seed outside the URDF limits:
`[-10, 10]` rad for joints 1–5 and `[-3.14, 3.14]` rad for joint 6. These are
RoboDojo simulation asset limits, not a certification of physical X5 limits.

The default solver takes at most 200 differential IK iterations. Each candidate
is checked against position error `1e-5` m, orientation error `1e-4` rad, and
joint bounds after numerical clipping. The result contains all seven coordinates
on success. Failure returns `IKResult(False, reason=...)` without a joint target.
Stagnation, iteration exhaustion and invalid seed limits have separate reasons.
The next call starts from its supplied seed, including after a failed call.

This is a local pose solve. A nearby seed improves branch continuity; it does
not guarantee a solution from an arbitrary seed, collision avoidance or dynamic
feasibility. No cuRobo collision/planning behavior is claimed. Mutable solver
state is protected by a per-model lock; separate arm groups own separate models.

## Geometry source and validation

`description.urdf` is a kinematic derivative of `Assets/Robots/x5/X5A.urdf` from
[RoboDojo-Benchmark/RoboDojo](https://huggingface.co/datasets/RoboDojo-Benchmark/RoboDojo),
revision `43dacb13d3051a9ccd421f4e7e08e4e3eb29dfab` (Apache-2.0). The source SHA256
is `7aaa308a1a5b6a213d914db86ba09ddd6252f25cb498ac9a148bfa6c7166a423`.
Camera branches and visual/collision geometry were removed; joint transforms,
limits and inertial data were retained. The solver further omits both fingers
from its six-DOF optimization problem. See the repository third-party notices.

The AsyncSim workspace includes `tests/test_arx_x5_kinematics.py`,
`scripts/capture_arx_x5_isaac.py` and `scripts/validate_arx_x5_kinematics.py`.
They exercise the public assembly, local seeded IK, failure recovery, independent
source URDF/USD forward chains and measured PhysX state. Reproduction commands
and the 2026-10-08 evidence are recorded in
`AsyncSim/docs/ARX_X5_KINEMATICS_REPORT.zh-CN.md`.
