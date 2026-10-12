# Runtime, Viewer and configuration ownership

Paths are relative to the repository root.

## Inference algorithm versus executor

`manimux/runtime/asyncsim.py` is a separate simulation runtime. It owns one
AsyncSim WebSocket session, reads a single atomic snapshot, and sends one
canonical joint target per control step. Its robot proxy is structural rather
than a `RobotBase` subclass, because `RobotBase` assembles physical arms,
controllers and kinematics. The offline fake-policy path supports direct and
smooth execution and the default, ACT, RTC, PAINT and DVAC strategies.
`run_policy()` composes the existing policy worker, adapter and strategy; the
`xpolicylab_joint_template.yaml` is not runnable with a real task until the
RoboDojo state/control layout and checkpoint identity are verified. Specialized
sampling hooks have only been exercised with fake policy capabilities; AAC and
process/independent decoding remain unsupported by this runtime. RoboDojo's
actual rollout has not been verified against a checkpoint.
Do not route an `asyncsim` experiment through
`EdgeRuntime` or the physical robot factory.

With `run.simulation_time_timeline: true`, command sampling/submission is gated by
`robot.control_hz` periods of simulator time. Wall time paces polling and enforces
real timeouts; missed periods are logged and skipped. Worker startup precedes the
episode reset so it does not consume evaluation time. Fixed-period RoboDojo task
evaluation continues while inference or safe hold keeps the current target.

`run.observation_streams` selects additional visible AsyncSim packets from the same
atomic snapshot. `ObservationSnapshot.stream_packets` preserves their source
identity and capture/availability timestamps for policy adapters. The canonical
`state` remains measured executor feedback; an adapter may build a separate policy
view when the training observation uses different semantics, such as a locally sent
gripper control target. Model-specific mapping stays in the policy adapter.

### Intermediate observation history

Use the existing `inference.strategy` decorator pattern for history, as UMI and
DP already do. Both Edge and AsyncSim pass visible observations, controller time
and the local timeline into `build_submission`; accepted/rejected plans and reset
use the existing strategy callbacks. No new PolicyAdapter lifecycle interfaces
are needed. Model-specific mapping and history remain in `policy_adapter/`.

The VPP2 decorator samples one visible observation per nominal 40 ms plan period,
keeps at most 24 pending samples and never backfills missing periods. Repeated
reads or frames are not automatically extra steps. Local plans do not prove
command dispatch, application, convergence or success. No simulator execution
metrics are policy inputs. `prepare_request` explicitly carries the history in a
picklable request; RPCs and native model history remain in the worker/backend.
Stateful request failure or an in-session history reset requires a new backend
session rather than silently replaying observations.

`run.control_period_alignment: episode` retains the default simulation-period
gating and interpolated reference sampling. The optional `plan` setting aligns
command periods to each committed plan and selects discrete native rows. It
requires serial simulation-time scheduling, a direct executor, zero commit lead,
and equal policy/control intervals. This preserves row identity for policies with
per-action history; missed periods are still skipped, never replayed in a burst.
In this mode command ACK `control_slot` is plan-relative and `control_origin_ns`
identifies its origin in the evaluation log. Neither is sent to the backend.

The VPP2 caller is documented in `docs/vpp2-arx-x5-runbook.md`. It supplies an
episode anchor and 24 observations per subsequent request through the existing
worker and WebSocket CALL protocol, leaving native history on the VPP2 backend.

Read `manimux/runtime/inference.py` (`InferenceStrategy`,
`build_inference_strategy`) and the selected strategy, such as
`runtime/rtc/strategy.py`. A new inference algorithm controls when to submit,
sampling requirements, chunk preparation, commit settings and response feedback.
Use those hooks and their existing strategy factory; do not imply every factory
supports arbitrary plugins or add algorithm-specific branches to adapters.

`manimux/runtime/timeline.py` owns `ActionTimeline`: committing and sampling decoded
joint trajectories on the runtime clock. Keep chunk handoff here and in the
strategy. Do not fake measured state or modify observations to implement another
algorithm's overlap/truncation behavior.

Read `manimux/runtime/executors/base.py`: an executor implements `reset(state)`,
`horizon_steps` and `step(now_ns, state, reference) -> RobotCommand`. It consumes
`ActionHorizon` and owns command generation/smoothing/configured limits, not model
requests or hardware sessions. Follow the actual executor construction in
`runtime/edge.py` when adding a type; do not assume the policy plugin loader applies.
Per-group gripper indices come from assembly layout, not a universal last-column
assumption. Reject unsupported behavior rather than silently choosing another mode.

For scheduling changes use a deterministic clock and synthetic chunks to check
timestamps, overlap, delayed/rejected responses and reset as relevant. For executor
changes check actual arm/tool limits and output groups. Preserve capability checks;
an algorithm setting alone does not implement the model's sampling hooks.

## Viewer, replay and recording

Read `manimux/viewer/robots/base.py` (`RobotAdapter`) and its sibling loader.
This is a display adapter, distinct from `policy_adapter.PolicyAdapter`.
Robot display implementations belong in `viewer/robots/`; generic UI consumes
their group definitions, pose/visual mappings and assets. The loader supports
`module:factory` and `manimux.viewer.robots` entry points.

Keep URDF display mapping distinct from command coordinates: rendering two fingers
does not add two action DOFs. Use offline models/assets for replay; loading an action
trajectory must not open a robot connection. Missing assets should be reported,
not substituted with invented geometry. Replay should not change live inference,
recording or session behavior. See `tests/unit/test_action_replay.py`.

Record runtime evidence under `manimux/recording/`. Distinguish predicted actions,
sent commands and measured motion; Viewer refresh rate is not control frequency.
Do not reintroduce teleoperation or demonstration collection as part of replay.

## Which YAML owns the setting?

The composition entry is an experiment under
`manimux/configs/experiments/<task>/<model>/`. Use `manimux.cli.load_config()` and
the existing referenced-config loading; do not create another schema/merge system.

- **Component YAML**, `configs/embodiment/{arm,end_effector,sensor}/`: implementation,
  component model options, action layout and reusable device defaults.
- **Robot assembly**, `configs/embodiment/robot/`: named components/groups, mounts
  and shared body hardware/control configuration. Shared control profiles own
  reusable motion limits; an experiment selects or explicitly overrides them.
- **Private station**, `configs/local/station.yaml` or selected `--local`: physical
  CAN/controller/device bindings and service addresses. Never assume enumeration
  order establishes left/right; do not commit installation-specific bindings.
- **Camera set**, `configs/embodiment/sensor/cameras/`: served source names and
  component selections, referenced by `camera_server.config`. Runtime `sensors`
  selects which sources to read; `policy.adapter.camera_map` maps them to model keys.
- **Deployment recipe**, `configs/policy/<model>/<embodiment>/<task>/`: checkpoint
  and serving/inference settings. `policy_server.config` selects this recipe;
  it does not own robot execution. Model defaults remain in XPolicyLab.
- **Experiment `policy`**: runtime client (`worker`, service/options), action
  interval/horizon/timeouts/delay and inline `adapter` class/mappings. There is no
  required separate YAML for every field. Do not hard-code experiment values such
  as `action_dt_s`, `horizon_policy_steps` or `inference_delay_s` into Python bases.
- **Inference YAML**, `configs/inference/`: algorithm request/chunk rules.
- **Executor YAML**, `configs/executor/`: smoothing and command-generation choices.
- **Experiment run/viewer/recording**: task, output, lifecycle/UI and rollout settings.
  Private training work stays in the ignored root `training/` workspace.

In the list above, `configs/` means `manimux/configs/`.
`policy` describes the runtime side; `policy_server` selects the model-serving side
of the same experiment. Pair their action representation, checkpoint identity and
sampler capabilities. Do not delete identity checks to make a mismatched pair start.

Keep `policy.action_dt_s` (model trajectory spacing), `robot.control_hz` (command
frequency), camera acquisition rate and recording video rate independent. Horizon,
decoded prefix and executed chunk length are also distinct. Do not infer one from
another just because one example happens to use the same value.

## Example and checks

Trace `configs/experiments/put_bottles/pi05/yam_pi05_joint.yaml` under `manimux/`:
robot assembly -> component YAMLs; camera set -> station bindings; policy server
recipe -> client format and joint adapter; RTC -> timeline; smooth executor ->
robot commands. Copy only the relevant choices when adding an experiment.

Load the changed YAML through the real loader and inspect its resolved values
without constructing devices or starting services. Check reference paths relative
to their containing file, client/adapter pairing and declared group layout. Do not
treat config loading as SDK, model or hardware validation.
