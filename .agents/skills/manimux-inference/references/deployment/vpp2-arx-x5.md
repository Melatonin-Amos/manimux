# VPP2 on AsyncSim / RoboDojo X5

## Ownership and configuration

Use `manimux/configs/experiments/asyncsim/robodojo_vpp2_insert_tubes.yaml` and
`manimux/configs/local/asyncsim_vpp2.example.yaml`. Model source and serving remain
the official `XPolicyLab/policy/VPP2` implementation at XPolicyLab revision
`125d5de` (official update `a647f3d`), with its pinned upstream `ed398648`.
No RoboDojo, AsyncSim core or XPolicyLab source changes are required.

The adapter `policy_adapter/vpp2/arx_x5.py` owns observation mapping, request
packing and X5 pose-to-joint conversion using the configured offline arm geometry.
`policy_adapter/vpp2/history.py` decorates the existing serial inference strategy
through `inference.strategy`, following the UMI/DP history pattern. Its inputs
are visible snapshots, measured capture timestamps and the local action timeline.
The shared scheduler calls `observe_snapshot(snapshot)` before request admission.
Both EdgeRuntime and AsyncSimRuntime already call this strategy interface; no
new PolicyAdapter hooks, public types or EdgeRuntime changes are needed.
`policies/xpolicylab/vpp2.py` is a thin client selected through the
existing policy factory and spawned `PolicyWorkerClient`. It uses XPolicyLab's
public CALL protocol, not a separate worker, server or transport.

The policy uses only three RGB cameras, measured joint feedback, configured robot
geometry, fixed base calibration and the controller-owned action plan. Link6
poses come from measured joints via FK and the calibrated base transforms, never
from simulator pose queries. A real robot can obtain these transforms through
installation calibration. The example base poses describe the RoboDojo layout;
replace them with measured installation values for another workspace.

Gripper conditioning explicitly uses `nominal_plan_target`: the normalized
opening in the corresponding row of the local committed plan, with measured
opening as the bootstrap before the first plan. This differs from a measured
opening, last sent target or simulator effective target; the current shared
strategy API does not report actual dispatch. It does not assert actuator
application under network/control delay. Canonical joint feedback stays measured for execution and
IK. The VPP2-specific snapshot carries the model view separately. Returned
absolute `[xyz, wxyz, gripper]` poses are transformed into arm bases once for IK.
RGB composition, resizing, normalization, video conditioning, sampling and native
model history remain in the official backend.

## Timing and history

The backend predicts 32 actions and returns its first 24 for execution at 25 Hz
(40 ms). Use default sampling, serial inference, a direct executor, `inference.algorithm: serial`, no blending
and `first_step_when_ready`. The history strategy validates the common runtime
requirements and inline decoding. The AsyncSim example additionally selects
`run.control_period_alignment: plan` for discrete native command rows. This is an
optional simulation scheduling setting, not an input to the adapter or history
plugin; the plugin itself has no simulator dependency.

Each request carries one anchor on the first inference, then exactly 24 new
observations after each complete nominal plan prefix. One visible observation is
sampled after each row's planned 40 ms control period using measured-state timestamps. Repeated control-tick calls are
deduplicated by local row/period, not camera sequence: delayed cameras may supply
the same source frame for different actions. Ordinary state/camera sequences and
capture timestamps travel through the spawned worker in `additional_info.sensor_sources`.
No simulator context, hidden pose, effective target, application time, expiry or
success information is delivered to the model.

The final boundary is reconciled with the next inference request's observation,
without appending an extra frame. The strategy never backfills earlier periods
from future observations. A missed sampling boundary or changed/trimmed plan stops
the rollout explicitly. Sampling is based on the controller's planned clock,
not proof of actuator application or physical convergence.

The history strategy buffers at most 24 observations and a pending request. It
keeps delivery data pending until a valid response and plan commit. Backend
history uses the official bounded retention and stride 25. During inference the
environment continues advancing under the held target; those hold periods are
not extra native actions. Task evaluation still advances independently.

AsyncSim uses simulator time internally as its controller clock so simulation
speed cannot change native action durations. Neither that clock nor simulator
execution metrics are model inputs. Metrics and task success remain in the final
evaluation report only; completion or success is RoboDojo's decision.

The client follows `reset`, tagged `reset_client`, history `update_obs`, tagged
`get_action`. Both update and action calls carry the same client identity, matching
the official deployment loop. Nested history contains plain RGB arrays because
the upstream image decoder handles top-level vision only. A partial call timeout
can already have updated model history: automatic retries are disabled until a
new session reset. Pause/Home can reset scheduling without resetting the model,
so continuing after an in-session history reset requires a new backend session.
Backend client retention and eviction remain backend-owned.

## Policy Server

The paired `joint2b_s100000` bundle needs `action.pt`, `video.pt`,
`dataset_stats.json`, its manifest and the Wan2.1 encoders/tokenizer. Video-only
Stage1/Stage2 checkpoints are not executable action policies. Follow the official
VPP2 installation/download documentation using the model's own uv environment.
The retained `env_cfg/arx_x5.yml` and robot metadata beside XPolicyLab are
also required by its legacy environment-name lookup.

From the ManiMux checkout, after the model environment and assets are installed:

```bash
XPolicyLab/policy/VPP2/.venv/bin/python \
  XPolicyLab/policy/VPP2/launch_policy.py \
  --task insert_tubes --port 19960 --replan 24 --seed 1 \
  --bundle /path/to/joint2b_s100000 --wan /path/to/Wan2.1-I2V-14B-480P
```

The official launcher validates the bundle and starts the existing server.
The ManiMux recipe checks policy/task/action representation and prediction
horizon via the handshake; the wrapper separately checks the returned 24-row
prefix. The upstream handshake does not expose a checkpoint hash or replan count,
so it cannot verify those fields. Keep the official launcher asset checks enabled.

Use the existing AsyncSim benchmark entry point with this experiment and
`async_config/profiles/manimux_zero_delay.yml`. Run the controller with
`envs/arx_x5/.venv/bin/python`, model inference in its own uv environment, and
simulation in the RoboDojo environment. No system Python is needed.

## Verified Boundary

```bash
# From the AsyncSim root, using the controller environment:
manimux/envs/arx_x5/.venv/bin/python -m pytest -q tests/test_vpp2_arx_x5.py
```

Tests exercise the actual configuration loader, adapter, IK, runtime, spawned
worker, thin client, WebSocket codec/server and official VPP2 `Model` interface.
Only the numerical inference backend is replaced. Three inference boundaries
record native history steps 0, 24 and 48 with no alignment mismatches. Tests also
cover repeated camera frames/control ticks, final-read reconciliation, session
resets, missing sampling boundaries, history retention until plan acceptance,
partial RPC failure and rejection of simulator data dependencies. The same
strategy contract is also exercised without simulator endpoints or run settings.

The server-side test uses the existing Pi05 uv environment for its protocol
dependencies; it does not load Pi05 or VPP2 weights. This is interface validation,
not real VPP2 forward inference or a successful RoboDojo rollout. The VPP2
environment and assets have not been installed/downloaded by this change.

## Hardware prerequisites

The observation and history contract uses information available on a real robot.
This does not yet make this AsyncSim experiment a hardware deployment: the
configured X5 arms are offline geometry models. A hardware driver, camera/robot
calibration, normalized gripper feedback mapping and actual EdgeRuntime hardware
validation are still needed before running on a physical X5. The strategy plugin
uses EdgeRuntime's existing calls and needs no additional notification wiring.
