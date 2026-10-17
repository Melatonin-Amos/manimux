"""File-relative experiment composition, without constructing runtime components."""

from __future__ import annotations

import hashlib
from copy import deepcopy
from pathlib import Path

import yaml


def compose(path: str | Path, stack: tuple[Path, ...] = ()) -> tuple[dict, dict[str, str]]:
    from manimux.cli import _merge

    source = Path(path).expanduser().resolve()
    if source in stack:
        raise ValueError(f"Configuration inheritance cycle: {(*stack, source)}")
    if len(stack) >= 4:
        raise ValueError("Configuration inheritance is limited to four files per chain")
    content = source.read_bytes()
    raw = yaml.safe_load(content)
    if not isinstance(raw, dict):
        raise ValueError(f"Configuration must be a mapping: {source}")
    bases = raw.pop("_base_", [])
    if isinstance(bases, str):
        bases = [bases]
    if not isinstance(bases, list) or any(not isinstance(p, str) for p in bases):
        raise ValueError(f"_base_ must be a path or list of paths: {source}")
    result, sources = {}, {}
    for reference in bases:
        base, dependencies = compose(source.parent / reference, (*stack, source))
        result = _merge(result, base)
        sources.update(dependencies)

    def resolve(value):
        if isinstance(value, list):
            return [resolve(v) for v in value]
        if not isinstance(value, dict):
            return value
        return {
            key: str((source.parent / Path(v).expanduser()).resolve())
            if key in {"config", "control_profile", "local"} and isinstance(v, str)
            else resolve(v)
            for key, v in value.items()
        }

    sources[str(source)] = hashlib.sha256(content).hexdigest()
    return _merge(result, resolve(deepcopy(raw))), sources


def apply_initial_pose(raw: dict) -> None:
    """Resolve a named robot pose as part of the experiment, not its collection."""
    import math

    from manimux.cli import read_yaml

    selection = raw.pop("initial_pose", None)
    if selection is None:
        return
    if not isinstance(selection, dict) or set(selection) != {"config", "id"}:
        raise ValueError("initial_pose requires config and id")
    path = Path(selection["config"])
    poses = read_yaml(path)["poses"]
    pose = poses[selection["id"]]
    options = raw["robot"].setdefault("options", {})
    if not options.get("move_to_start_on_connect"):
        raise ValueError("Initial pose requires move_to_start_on_connect")
    joints = pose["joints"]
    dimensions = raw["robot"].get("group_dims", {})
    if set(joints) != set(dimensions):
        raise ValueError("Initial pose must specify every robot group")
    for group, size in dimensions.items():
        q = joints[group]
        if (
            not isinstance(q, list)
            or len(q) != size
            or any(
                isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v)
                for v in q
            )
        ):
            raise ValueError(f"Invalid initial pose joints: {group}")
    duration = pose.get("duration_s", options.get("start_duration_s"))
    if (
        isinstance(duration, bool)
        or not isinstance(duration, (float, int))
        or not math.isfinite(duration)
        or duration <= 0
    ):
        raise ValueError("Initial pose duration must be finite and positive")
    options.update(start_joints=deepcopy(joints), start_duration_s=duration)
    raw.setdefault("run", {})["initial_pose"] = {
        "id": selection["id"],
        "label": pose.get("label", selection["id"]),
        "joints": deepcopy(joints),
        "duration_s": duration,
        "source": deepcopy(pose.get("source")),
    }
    raw.setdefault("config_sources", {})[str(path)] = hashlib.sha256(path.read_bytes()).hexdigest()
