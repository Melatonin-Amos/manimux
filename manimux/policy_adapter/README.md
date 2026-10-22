# Policy adapters

This package owns observation preparation and conversion of policy-service actions
into ManiMux `ActionChunk`s. Experiments select an actual Python implementation;
there is no parallel model-specific integration package.

```yaml
policy:
  worker: xpolicylab_ws
  adapter:
    type: manimux.policy_adapter.joint:JointAdapter
    group_order: [left_arm, right_arm]
    group_prefixes: {left_arm: left, right_arm: right}
    gripper_dofs: 1
    camera_map:
      cam_head: front_camera
      cam_left_wrist: left_camera
      cam_right_wrist: right_camera
  action_dt_s: 0.03333333333333333
  horizon_policy_steps: 50
  options:
    action_format: joint
    server: ws://127.0.0.1:8500
```

`policy.adapter` holds semantic mappings; `policy.options` holds transport/backend
options. Timing and horizon are declared once in `policy`. The wire codec reads the
same camera/group mapping when serializing a request.

The factory calls `Adapter(robot, policy, kinematics=...)`. The runtime can provide
its assembled robot's offline kinematics. Decoder processes never receive live
hardware handles. Specialized solvers and their configured tolerances are preserved
by this directory migration; consolidating them must preserve their TCP/IK behavior.

- `base.py`: one interface, identity observation/request hooks and abstract
  `decode_action(raw, context)`. The base assumes no joint action format.
- `joint_mount.py`: `JointAdapter` for a joint-space model trained with its arm bases at a
  different mount (`base_offset_m`); converts state, RTC/PAINT rows and actions by FK/translate/IK.
- `joint.py`: canonical grouped absolute joint targets to timed trajectories. Arm values
  precede gripper values; units remain unchanged. No normalization, delta recovery,
  IK or smoothing is performed here.
- `sapolicy/`, `openwam/`, `umi_dp/`, `xr1/`, `lingbot_vla2/`: specialized pose,
  observation history and action conversion implementations.

One model may select multiple adapters when its service exposes different contracts.
Multiple models may share an adapter when their contracts agree. A new checkpoint
alone does not need a new class. Put mappings in configuration and conversion
procedures in Python, documenting units, coordinate frames, delta anchors and time.

Model loading, normalization and model sampling remain in XPolicyLab. Transport
remains in `policies/`; scheduling in `runtime/`; command generation in executors.
No empty generic TCP adapter is provided: existing pose adapters differ in anchor,
quaternion, timing and IK behavior and cannot be merged solely because they use poses.

## RoboDojo Runtime Motor Control

`openwam.motor_eef:OpenWAMMotorEEFAdapter` is an opt-in AsyncSim adapter for
OpenWAM's absolute EEF contract. It preserves source EEF rows while an
adapter-provided `RoboDojoMotorController` performs per-row measured-state IK and
native-compatible ten-point interpolation in an isolated client-owned worker.
The existing `AsyncSimRuntime` loop samples physical motor targets at 250 Hz;
the simulator applies them without another IK/interpolation or gripper mapping.

Use `configs/experiments/asyncsim/robodojo_openwam_insert_key_motor.yaml` with an
AsyncSim server exposing `proprio.motor_joint_state` and eight-coordinate
physical motor groups. `robot.options.motor_control_config` points to a JSON
export of the actual robot's joint order, bases, cuRobo paths, gripper parameters
and solver Python executable. The AsyncSim server can export this metadata with
`--control-chain motor --export-motor-config PATH`.

This mode keeps normal independent sensor streams and continuous physics during
both inference and IK. It requires neither fresh aligned observations nor a
simulation-pause interface. Serial, unblended 25 Hz source chunks and 250 Hz
motor commands are currently supported; incompatible scheduling is rejected.
Existing adapters do not enable this command controller by default.
