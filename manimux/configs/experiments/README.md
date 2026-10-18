# Experiment variants

Only `pi05-latest300ep-step30000/` is published in this checkout. Other experiment
directories and their config list entries are local and ignored by Git. Creating
a local study does not publish it; explicitly update the allowlist in `.gitignore`
when another study is intended for sharing. Shared component/task definitions are
maintained separately from these installation-specific experiment variants.
Task definitions needed only by historical local variants live in the ignored
`../local/tasks/` directory; the public `../tasks/` directory contains shared task
definitions rather than migration-generated legacy copies.

Store reusable variants under `<model-checkpoint>/<task>/<variant>.yaml`.
Use `_base_.yaml` only for shared choices, and inherit task definitions from
`../../tasks/` at the appropriate relative depth. Component references stay in
`policy/`, `inference/`, `embodiment/` and `executor/`. Scoring lives inline
in the shared task YAML.

The task YAML also owns `run.max_control_steps`. Model bases and concrete methods
inherit that limit; do not copy it into each variant. Chunk execution windows
remain inference settings and are independent of the total rollout budget.

Variants used in formal studies declare `experiment.task` and `experiment.variant`,
without an output root or repeat budget. A file in `../lists/` selects variants,
trials and `recording.root`; launch all services with that same `--config-list`.

The same variant can be tried directly with `run --config <variant>`. Camera and
generic policy services also accept that `--config`. This route offers Prepare Run
only and saves separate trial records; it never consumes the study repeat budget.
A new standalone runtime config needs no experiment identity, layouts or scoring.

Current Pi05 entry: [`../lists/pi05-latest300ep-step30000.yaml`](../lists/pi05-latest300ep-step30000.yaml).
See the [complete protocol and commands](../../../guideline.html#experiment-lists).

Unresolved historical recipes live in ignored `../local/archived_experiments/`;
they are not runnable examples or config list entries.

Raw historical sessions retain their saved parameters and paths. Their config list
`history` inputs require compatible conditions. A change only to the stop budget
retains progress without renaming variants or rewriting sessions; each session's
saved budget and condition fingerprint remain authoritative for analysis.
Sessions containing only a startup manifest and no `rollout-*` records do not
block a corrected configuration. Their manifests remain unchanged; even a partial
rollout keeps the normal compatibility check.
