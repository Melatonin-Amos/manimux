# Service entry points

A study selects one collection and one private station. Camera, model and runtime
resolve that same entry; use the [launch guide](../../guideline.html#latest300ep-launch)
for commands and environment selection.

| Module | Owns |
| --- | --- |
| `manimux.servers.camera.server --collection` | Physical cameras and image transport |
| `manimux.servers.policy --collection` | The selected XPolicyLab deployment in its model environment |
| `manimux serve --collection` | Experiment switching, scheduling, execution and recordings |
| `manimux.robogui.dashboard` | Operator controls and visualization |

`--experiment-id` selects a service configuration when a collection contains
incompatible entries. Without it, ambiguity is rejected. Switching runtime entries
does not reload model weights or reconnect a different camera set automatically.

Model-specific launchers remain available for artifact validation and standalone
model debugging. Their `--config` selects a **model recipe**, not an experiment or
recording root. Generic policy `--check` resolves configuration only; it does not
load weights or prove inference readiness. Camera clients never open physical cameras.

OpenWAM artifact export requires both `--bind-runtime-config <output>` and
`--runtime-template <existing-experiment>`; it has no dependency on an ignored
private example. Model networks, normalization and samplers remain in XPolicyLab.
