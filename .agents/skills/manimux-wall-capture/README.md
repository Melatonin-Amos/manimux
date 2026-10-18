# ManiMux wall footage — contributor kit

Give this complete folder (or its ZIP) to a colleague. It contains a self-contained
agent skill, capture guidance, an optional X11 recorder, and a validator/packager.
It has no model weights, footage, host credentials, or absolute station paths.

## Use with a local agent

Extract it, then place `manimux-wall-capture/` under the local ManiMux project's
`.agents/skills/` if the agent supports that convention. If it uses another skill
location, use that location. It can also simply read the extracted `SKILL.md`.
Do not overwrite a different installed version without checking it.

Paste this prompt into the colleague's agent:

> Read the attached/extracted manimux-wall-capture/SKILL.md and apply it to this
> local ManiMux station. Inspect our actual robot, model, inference configuration,
> GUI and available recorder. Help us produce one or two clean 30–60 second RoboGUI
> clips for the ManiMux multi-embodiment video wall, with a continuous 12–20 second
> nominated execution interval in each. Use 1080p30 when smooth, or 720p30 if needed.
> Coordinate physical actions with the operator; do not start robot motion yourself
> unless explicitly authorized. Record actual provenance and any limitations, then
> validate and package the submission ZIP. Return the ZIP path for me to forward.

The agent should adapt to the local checkout and tools. It must not assume the
coordinator's port, YAM embodiment, interpreter or managed CLI exists here.
The local operator runs the task. The agent handles recording and packaging.

## What to return

One station-labelled ZIP produced by `scripts/submission.py pack`: clean GUI clips,
manifest, technical validation report and SHA-256 list. Optional synchronized
external camera and human evaluation/save footage may be included. Keep originals
locally. Send the ZIP to the coordinator; the coordinator will aggregate them for
the final editor. No rendering of the wall is required from contributors.

Python 3 and FFmpeg/ffprobe are required for the helper scripts. The recording
method itself is chosen for the local OS; X11 helper use is optional. No special
agent connector or repository-specific Python environment is required for packaging.
