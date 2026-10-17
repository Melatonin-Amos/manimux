# Experiment variants

Only `pi05-latest300ep-step30000/` is published in this checkout. Other experiment
directories and their collection entries are local and ignored by Git. Creating
a local study does not publish it; explicitly update the allowlist in `.gitignore`
when another study is intended for sharing. Shared component/task definitions are
maintained separately from these installation-specific experiment variants.

Store reusable variants under `<model-checkpoint>/<task>/<variant>.yaml`.
Use `_base_.yaml` only for shared choices, and inherit task definitions from
`../../tasks/` at the appropriate relative depth. Component references stay in
`policy/`, `inference/`, `embodiment/`, `executor/` and `evaluation/`.

Each variant declares `experiment.task` and `experiment.variant`. It has no output
root or repeat budget. A file in `../collections/` selects variants, trials and
`recording.root`; launch camera, model and runtime through that same collection.

Current Pi05 entry: [`../collections/pi05-latest300ep-step30000.yaml`](../collections/pi05-latest300ep-step30000.yaml).
See the [complete protocol and commands](../../../guideline.html#experiment-lists).

Raw historical sessions retain their saved parameters and paths. Their collection
`history` inputs are accepted only when saved conditions match the selected variant.
