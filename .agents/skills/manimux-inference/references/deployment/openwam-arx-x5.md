# Official OpenWAM RoboDojo X5 Deployment

This deployment reuses the vendored OpenWAM backend and the shared XPolicyLab
WebSocket server. ManiMux owns observation selection, embodiment conversion,
serial chunk scheduling and canonical joint commands. It does not change the
model, normalization, sampler, RoboDojo tasks or reward conditions.

## Validation Status

On 2026-10-09 the official `OpenWAM/OpenWAM-Alpha-Sim-RoboDojo` release at
`2c1302294e3ba8319bbdb2c803b7a27de9292d03` was downloaded and hash-verified.
The community Cursor release is not used.

The independent OpenWAM environment passed dependency checks, real CUDA tensor
operations on RTX 5090 and public backend imports. Nine new adapter tests,
ten X5 geometry tests, eight XR1 regression tests and 28 AsyncSim runtime tests
passed. Actual `insert_key`, seed 0 observations supplied three nonblank
480x640 RGB images, environment-relative link6 poses and gripper control state.
The adapter's FK/base placement differed from Isaac by about 0.03 mm and
0.000302 rad in that snapshot. No action was submitted during this check.
An offline 32-step round trip using those recorded current poses also passed IK;
these geometry-test targets were not model predictions and were never executed.

**Real model loading failed before GPU transfer.** The kernel OOM-killer killed
the process after 46.5 seconds, with maximum RSS 22,762,832 KiB, approximately
21.71 GiB. GPU usage remained 21 MiB. This was host RAM exhaustion, not a GPU
driver error or a measured CUDA OOM. Real forward, policy-server readiness,
closed-loop rollout and task success remain unverified.

The process had a 25 GiB RAM / 2 GiB swap cgroup limit, but the kernel reported
`global_oom`, not a cgroup-limit OOM. Do not repeat the same load while other
processes consume the machine's approximately 30 GiB RAM. The config-driven
Wan loader currently constructs components on CPU before moving loaded weights
to CUDA; its deployment entry has no low-memory initialization switch for this
checkpoint. Using a larger-RAM host is possible. A meta/strict-assignment loading
fix would be a separate upstream change requiring approval, not an implicit
patch in this embodiment adapter or launcher. GPU inference and simultaneous
Isaac/model memory requirements still need measurement afterward.

Workspace evidence is under `../async_eval_result/openwam_validation/` relative
to the ManiMux checkout: `model_server.log`, `model_loading_failure.json`,
`simulator.log`, `real_observation_validation.json`, `real_observation.npz` and
three `real_*.png` files. `real_ik_contract_validation.json` records the offline
IK check. The check produced no rollout result or rollout video.

## Observation and Action Contract

- Robot: simulated dual ARX X5, selected by `robodojo_x5_dual.yaml`; each canonical group contains six joint angles
  in radians and one normalized gripper coordinate.
- Model observations: RGB `cam_head`, `cam_left_wrist`, `cam_right_wrist`, the
  visible language instruction and `proprio.ee_pose` / `proprio.joint_state`
  packets from the same AsyncSim snapshot. Poses use environment-origin-relative
  position and world orientation, quaternion wxyz, endpoint `link6`.
- The policy view uses the original RoboDojo gripper control opening, 0 closed
  and 1 open. Executor feedback remains the measured canonical gripper value.
- The backend performs its own EEF20/base-frame conversion and normalization.
  ManiMux must not normalize again, apply XR1's MiBot reframe or add a TCP bias.
- XPolicyLab returns absolute environment-relative EE poses. The existing client
  decodes them to grouped `pose` rows; the X5 adapter converts each pose to its
  arm base and uses the configured offline X5 IK model. Any failed IK rejects
  the entire chunk. No dummy policy, identity bypass or hold-action replacement
  is used to report a successful inference.
- This recipe uses the full 32-step chunk, 40 ms waypoints, 25 Hz nominal control,
  serial inference, first-step-when-ready and zero injected sensor/command delay.
  Physics continues during inference; this is not the original paused synchronous
  simulator loop. RTC/PAINT and other specialized sampling modes are not enabled.
- Backend reset uses the shared client; changing sessions clears adapter anchors.
  Auxiliary packets retain episode, sequence, capture and availability metadata.

## Environment

From the AsyncSim workspace root, use a dedicated Python 3.12 environment.
The commands below reproduce the installed package setup; the checkpoint remains
unchanged. Normal package builds may create disposable `build/` directories next
to their project metadata; these are not adaptation source.

```bash
export UV=/tmp/asyncsim-uv-tools/uv
export UV_CACHE_DIR=/tmp/asyncsim-openwam-uv-cache
export MODEL_PYTHON="$PWD/manimux/envs/openwam/.venv/bin/python"

"$UV" venv --python "$PWD/manimux/envs/xr1/.venv/bin/python" manimux/envs/openwam/.venv
"$UV" pip install --python "$MODEL_PYTHON" \
  'torch==2.8.0' 'torchvision==0.23.0' \
  --index-url https://download.pytorch.org/whl/cu128
DS_BUILD_OPS=0 "$UV" pip install --python "$MODEL_PYTHON" \
  ./manimux/XPolicyLab ./manimux/XPolicyLab/policy/OpenWAM/OpenWAM \
  'torch==2.8.0+cu128' 'torchvision==0.23.0+cu128' 'numpy<2.5'
"$UV" pip check --python "$MODEL_PYTHON"
```

Use standard isolated DeepSpeed builds without `--no-build-isolation-package`:
that option failed on this machine because CUDA Toolkit/nvcc is not installed.
Training kernels are not needed for this inference path. The successful installed
versions include Torch 2.8.0+cu128, Transformers 5.19.0, Diffusers 0.41.0,
DeepSpeed 0.18.9 and NumPy 2.4.6. The existing XR1 environment was not modified.

The runtime/IK environment is independent:
`manimux/envs/arx_x5/.venv/bin/python`. It needs ManiMux's regular runtime,
`arx-x5-ik` and `xpolicylab` dependencies, not OpenWAM model dependencies.
The simulator uses the existing RoboDojo/Isaac environment.

## Startup After Resolving Loading Memory

The example station binds the downloaded official directory and port 18866.
The launcher validates the recipe, manifest, asset sizes, config/stat hashes and
the complete checkpoint hash before starting the existing shared server.
There is no CLI option to bypass weight verification or use a dummy model.

```bash
export EXPERIMENT=manimux/manimux/configs/experiments/asyncsim/robodojo_openwam_insert_key.yaml
export LOCAL=manimux/manimux/configs/local/asyncsim_openwam.example.yaml
export PROFILE=async_config/profiles/manimux_openwam_zero_delay.yml
export MODEL_PYTHON="$PWD/manimux/envs/openwam/.venv/bin/python"
export RUNNER_PYTHON="$PWD/manimux/envs/arx_x5/.venv/bin/python"
export PYTHONPATH="$PWD/manimux:$PWD"
export HF_HUB_OFFLINE=1 TRANSFORMERS_OFFLINE=1

"$RUNNER_PYTHON" -m manimux.servers.asyncsim_openwam \
  --experiment "$EXPERIMENT" --local "$LOCAL" --check
"$MODEL_PYTHON" -m manimux.servers.asyncsim_openwam \
  --experiment "$EXPERIMENT" --local "$LOCAL"
```

Only after the model loads and a real 32-step forward passes, start the simulator
in another terminal with matching profile, task and seed. Keep the model process
isolated from cameras and robot hardware:

```bash
/home/jw/miniconda3/envs/RoboDojo/bin/python scripts/run_robodojo_async_server.py \
  --task-name insert_key --env-cfg arx_x5 --profile "$PROFILE" \
  --eval-seed 0 --num-envs 1 --device-id 0 \
  --host 127.0.0.1 --port 18865 --tick-interval 0.004 --headless --enable_cameras
```

Then run the existing benchmark, not a parallel model-specific evaluation loop:

```bash
PYTHONPATH=manimux "$RUNNER_PYTHON" scripts/run_async_benchmark.py \
  --experiment "$EXPERIMENT" --local "$LOCAL" \
  --task insert_key --policy OpenWAM \
  --checkpoint OpenWAM-Alpha-Sim-RoboDojo-step60000-2c130229 \
  --profile "$PROFILE" --seeds 0 --timeout 1800 \
  --output async_eval_result/openwam_insert_key_zero_delay
```

Use unoccupied ports and update the matching station/experiment bindings; never
stop unrelated services. A finished `_result.json`, actual inference/command
audit and task termination are required before reporting a completed rollout.
Success must come from RoboDojo's original task result, not adapter tests.
