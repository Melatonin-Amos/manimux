"""Server-local options for XPolicyLab's existing AutoHorizon selector."""

from __future__ import annotations

import math
from collections.abc import Mapping
from contextvars import ContextVar
from dataclasses import dataclass
from functools import cache, wraps
from importlib import import_module

_DEFAULTS = {"hold_thr": 0.3, "run_len": 1, "max_entropy_q": 0.9}
_SELECTORS = {
    "Pi_05": "openpi.policies.policy",
    "Xiaomi_Robotics_1": "XPolicyLab.utils.autohorizon_official",
    "LingBot_VLA2": "XPolicyLab.utils.autohorizon_official",
    "OpenWAM": "XPolicyLab.utils.autohorizon_official",
}


@dataclass
class _Selection:
    options: dict
    used: bool = False


_selection: ContextVar[_Selection | None] = ContextVar("autohorizon_selection", default=None)


def validate_autohorizon(options: Mapping) -> dict:
    """Validate server YAML without importing model dependencies."""
    if not isinstance(options, Mapping) or options.keys() - _DEFAULTS.keys():
        raise ValueError("policy_server.autohorizon accepts only hold_thr, run_len, max_entropy_q")
    values = {**_DEFAULTS, **options}
    for key in ("hold_thr", "max_entropy_q"):
        value = values[key]
        if (
            isinstance(value, bool)
            or not isinstance(value, int | float)
            or not math.isfinite(value)
        ):
            raise ValueError(f"autohorizon.{key} must be a finite number")
    if values["hold_thr"] < 0:
        raise ValueError("autohorizon.hold_thr must be nonnegative")
    if not 0 <= values["max_entropy_q"] <= 1:
        raise ValueError("autohorizon.max_entropy_q must be in [0, 1]")
    run_len = values["run_len"]
    if isinstance(run_len, bool) or not isinstance(run_len, int) or run_len < 1:
        raise ValueError("autohorizon.run_len must be a positive integer")
    return values


@cache
def _install_selector(module_name: str) -> None:
    # Pi05 imports the function into its policy module; other models import the
    # shared selector at call time. Bind at those call sites, without editing files.
    module = import_module(module_name)
    original = module.bidir_soft_pointer

    @wraps(original)
    def select(attention, *args, **kwargs):
        active = _selection.get()
        if active is None:
            return original(attention, *args, **kwargs)
        # The pinned selector squeezes its convolution output; a full-horizon
        # run can become a scalar. Exclude that boundary instead of changing it.
        if active.options["run_len"] >= attention.shape[0]:
            raise ValueError("autohorizon.run_len must be smaller than the action horizon")
        result = original(attention, **active.options)
        active.used = True
        return result

    module.bidir_soft_pointer = select


def configure_autohorizon(model, policy_name: str, options: dict) -> None:
    """Bind validated options to this model's AutoHorizon calls only."""
    module_name = _SELECTORS.get(policy_name)
    if module_name is None or "autohorizon" not in model.sampling_modes():
        raise ValueError(f"AutoHorizon configuration is unsupported by {policy_name!r}")
    _install_selector(module_name)
    original = model.get_action_autohorizon
    # Snapshot once so caller mutations cannot change sampling or its metadata.
    options = dict(options)

    @wraps(original)
    def infer(sampling):
        active = _Selection(options)
        token = _selection.set(active)
        try:
            result = original(sampling)
        finally:
            _selection.reset(token)
        if not active.used:
            raise RuntimeError("Configured AutoHorizon selector was not called by the model")
        return {
            **result,
            "autohorizon": {
                **result["autohorizon"],
                "hold_threshold": options["hold_thr"],
                "run_length": options["run_len"],
                # Preserve the pinned selector's upper-quantile cap.
                "entropy_quantile": min(options["max_entropy_q"], 0.999),
            },
        }

    model.get_action_autohorizon = infer
