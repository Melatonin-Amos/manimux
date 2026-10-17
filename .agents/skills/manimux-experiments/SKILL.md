---
name: manimux-experiments
description: Build a custom ManiMux real-robot study from a supported deployment, configure tasks and scoring, collect traceable trials, and evaluate recordings. Use for new experiment setup, comparison protocols, progress, human scores, trajectory metrics and optional PRM reports.
---

# ManiMux Experiments

Work from the user's checkout. Read the [repository guide](../manimux-development/references/agent-guide.md)
before editing. This is the entry point for building or evaluating a user's own study;
use the development skill only when a required component or behavior is missing.

## Start from the user's experiment

Collect task requirements conversationally; do not ask users to understand or fill
YAML schemas. The agent writes and validates configuration from their answers.
Inspect existing configuration and decisions before asking questions. Resolve only
missing choices: robot/cameras, model/checkpoint and action contract, task, methods,
and output location. For scored comparisons also establish layouts/reset, repetitions,
order, stopping/invalid-attempt rules and metrics. Do not inherit this team's tasks,
checkpoints, ten-layout budget or scoring weights as defaults.

Establish whether the study is single-task or multi-task. If the conversation or
existing study does not make this clear, ask the user before choosing its entry point.
An already stated all-task study remains multi-task when the user switches models;
do not silently reduce its scope to the task named in a checkpoint or example YAML.
Checkpoint training coverage and requested evaluation tasks are separate facts.
If the requested multi-task weights are unavailable, report that and clarify the
checkpoint choice while preparing the requested catalog; do not substitute a
single-task launch command as though it satisfies the study.
Use a `--collection` entry point for formal studies, including single-task/single-
variant studies. Multiple tasks and methods appear in RoboGUI. Keep the same collection,
station and optional `--experiment-id` in camera, model and runtime commands. Different
service contracts may be listed together, but require explicit selection and service
restarts; compatible entries switch idle without restarting the model. Inspect through
`python -m manimux inspect --collection <path>` before giving launch commands.

Free inference needs no rubric, study register or PRM. For scoring, ask whether binary
success is sufficient or partial credit is wanted. Define observable outcomes and
whether final state or an achieved milestone counts; do not invent milestone weights.
Use an existing study register when supplied. This checkout's `experiments.md` is private
and optional; preserve it rather than recreating its results from memory.

## Interview the user, then author configuration

Reuse answers already supplied or verified in existing configs. Ask only unresolved
questions in small batches, using task language rather than internal field names.
Do not present the following checklist as one long form.

| Information to establish | Agent responsibility |
| --- | --- |
| Task and instruction | Ask what the robot should do. Preserve the checkpoint's expected prompt when known; do not silently translate or rewrite it. |
| Success and partial credit | Ask what observable outcome counts as success and whether intermediate stages need scores. Obtain stage definitions and scores; distinguish achieved milestones from final-state scoring. |
| Scene and starting pose | Establish layout/reference source, reset procedure and Home or a named robot pose. Reuse known presets; do not invent joint coordinates from prose or an image. |
| Trial boundaries | Establish timeout/stopping rules and what counts as an invalid trial versus a policy failure. |
| Collection plan | Establish tasks, methods, layout subset and repetitions. Repeats create rollouts, not copied config files. |
| Storage | Ask for the recording root when unknown, inspect existing progress, and explain resolved absolute output locations. Never silently change roots to bypass completed trials. |

For example, after the task and hardware are known: "What counts as complete? Do you
want binary scoring or partial scores? If partial, describe the stages and scores."
Ask about poses/layouts and collection size separately when still missing. Free runs
do not require an evaluation interview. Do not impose this team's stage scores,
layout count, repeat count or execution window on another user's study.

Once required answers are available, write files without another generic permission
round. Reuse component configs and scoring templates. Keep task semantics (prompt,
available layouts, rubric and default pose) separate from collection choices (layout
subset, repetitions, selected variants and recording root). Variants should express
deliberate differences rather than duplicate shared settings. Show a concise summary
of the task, rubric, pose, trial plan and output paths, link the authored files, and
validate through the actual loader. Keep unresolved required semantics explicit;
do not fabricate values to make validation pass.

The implemented protocol is documented in
[Collections and storage](../../../guideline.html#experiment-lists).
`manimux.configuration.compose` owns file-relative `_base_` composition;
`manimux.collections.load_collection` owns identity, trial plan, storage and history.
Do not create a second catalog format or suggest direct managed-variant launches.

Publication is explicit: this checkout tracks only the `pi05-latest300ep-step30000`
experiment group and its collection. Other studies remain local through `.gitignore`.
Do not force-add ignored studies. If the user requests publishing another study,
update the experiment and collection allowlists together and verify its dependencies.

## Build a new real-robot experiment

Follow the stages that remain unfinished; do not repeat an already working deployment.

1. **Bind the station.** Use [station setup](../manimux-station-setup/SKILL.md) for
   environments, devices and service addresses. Another installation of a supported
   robot needs local bindings, not a new driver. Carry out that setup: inspect local
   facts, ask only for unknown physical mappings/paths, write the private station file
   and validate it. Do not leave the user to fill a template or repeat known answers.
   Route unsupported hardware/models to
   [development](../manimux-development/SKILL.md).
2. **Select the policy contract.** Use [inference](../manimux-inference/SKILL.md) and
   the matching deployment reference. Confirm weights, normalization, camera mapping,
   state/action representation, action interval and horizon. Runtime, camera and model
   services must use the same experiment/station; do not substitute weights for a method.
3. **Write shared task and deliberate variants.** Put prompt, available layouts,
   scoring reference and optional named initial pose in `manimux/configs/tasks/`.
   Reuse shared `embodiment/initial_poses/` presets; never infer joint coordinates.
   Put variants in `experiments/<model-checkpoint>/<task>/`, with `_base_.yaml` for
   common choices only when useful. Declare `experiment.task` and `experiment.variant`.
   File-relative `_base_` supports mapping merge and list replacement. Keep inheritance
   shallow. Variants must not have `run.output_dir`.
4. **Create the collection.** Save `collections/<study>.yaml` with stable `id`, the
   user's `recording.root`, and entries `{id, config, label?, trials?}`. `trials` owns
   layout subsets and repetitions. Algorithm overrides belong in variants, never in
   collection entries. Reusing the collection restores its output path/progress.
   Changed scientific conditions require a new variant ID when records already exist.
   Version rubrics; retain saved old snapshots. Optional `history` imports individual
   old sessions only after saved conditions match. Never relocate raw data just to
   match a new config tree or count old 16-step trials as new 32-step trials.
5. **Validate and launch.** Inspect the collection, selected service recipes and output
   directories without constructing hardware. `inspect --collection ... --index`
   refreshes the derived inventory; camera and generic policy `--check` do not start
   devices/model. Deliver all service commands with the same `--collection`, station
   and explicit ID when service configuration differs. Use the model's environment.
   This Pi05 comparison uses fixed 32-step windows, not 16; other models use the
   agreed ratio. Report AutoHorizon separately as adaptive. Configuration work does
   not authorize service startup or motion.
6. **Collect and evaluate.** In serve mode select Task/Method/Layout → Prepare Run
   or Prepare Experiment → Activate Warmup → Start → Finish → save/skip evaluation.
   Prepare may connect/move the robot as configured. Each attempt gets a new rollout;
   reset the scene per protocol. Report generation is a separate offline step below.

Selection is idle-only after recovery/evaluation. The catalog is loaded at serve
startup: disk edits require a runtime restart; model/camera changes may also require
those services to restart. Never silently restart an active experiment. Templates
track layouts/repeats; they do not automatically randomize or schedule a campaign.

## Stable study storage

Follow [storage protocol](../../../guideline.html#study-storage-protocol). Reuse the
same config/output for service restarts; dates belong only to generated session IDs.
Name new settings by the changed scientific variable, not the day. Record a deliberate
independent replication explicitly instead of evading completed-repeat counters.
Before moving records, classify each saved manifest; keep adaptive selectors separate
from fixed windows and distinguish Async refill/progress and RTC beta. Move whole
sessions, preserve snapshots/labels, verify hashes and retain a path mapping. Historical
aliases must not be counted alongside canonical paths. Update README, guideline and
active catalog references when retiring launch entries. This user's Pi05 active protocol
uses K=32; old K=16 records are archival and must not be supplied as new launch commands.

## Evaluate saved trials

- [Evidence and reporting](references/evidence-and-reporting.md): locate sessions,
  qualify cohorts, preserve setting identities and update progress/result rows.
- [HTML reports](references/evaluation-reports.md): create `task.yaml`, bind episodes
  and settings, select metrics, and run `python -m manimux.evaluation --task-dir ...`.
  Includes NLDLJ/NSPARC configuration and interpretation; the report index is separate
  from the runtime scoring rubric. These trajectory metrics describe references.
- [Human and seam metrics](references/human-and-seam.md): rubric-compatible scores,
  continuous versus end-hold seams, eligibility and per-episode aggregation.
- [PRM](references/prm.md): optional judge profile, video manifest and GPU evaluation.
- [Records](references/records.md): recorded fields and evidence limitations.
- [Manipulation scope](../../../manimux/evaluation/manipulation_scope/SKILL.md): only
  when drawing demonstration/inference coverage on camera frames.

Use recorded config/rubric snapshots to identify a trial, not today's YAML or folder
name alone. Keep finalized, partial, invalid and unlabelled attempts distinguishable.
Never infer task success from normal runtime completion or delete failed trials to
improve a mean. Derive metrics outside raw episodes and save cohort/provenance plus
per-metric included counts and missing reasons. Missing results remain missing.

## Deliver

For setup: link the new experiment, station/model references, rubric/collection,
resolved timing and output paths; give validation results and the operation sequence.
For evaluation: link the versioned report and reproducible cohort, update only the
agreed study rows, and distinguish collected, evaluated and still-missing evidence.
