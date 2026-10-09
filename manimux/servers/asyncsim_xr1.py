"""Launch the existing XPolicyLab XR1 server paired with an AsyncSim experiment."""

from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path

from manimux.cli import read_experiment

REPO_ROOT = Path(__file__).resolve().parents[2]


def resolve_server(experiment, local):
    config = read_experiment(experiment, local=local)
    server = config["policy_server"]
    if (server["policy_name"], server["protocol"], server["output_format"]) != (
        "Xiaomi_Robotics_1", "ws", "packed_ee_delta"
    ):
        raise ValueError("AsyncSim XR1 requires the XPolicyLab packed_ee_delta recipe")
    root = Path(server["model_dir"])
    for name in ("config.py", "last.ckpt/checkpoint/mp_rank_00_model_states.pt"):
        if not (root / name).is_file():
            raise FileNotFoundError(f"XR1 checkpoint artifact missing: {root / name}")
    processor = Path(server["vlm_processor_path"])
    for name in ("config.json", "tokenizer.json", "preprocessor_config.json"):
        if not (processor / name).is_file():
            raise FileNotFoundError(f"XR1 processor artifact missing: {processor / name}")
    return server


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--experiment", type=Path, required=True)
    parser.add_argument("--local", type=Path, required=True)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    server = resolve_server(args.experiment, args.local)
    print(json.dumps(server, indent=2), flush=True)
    if args.check:
        return 0
    os.environ["XR1_QWEN_VL_CONFIG_SOURCE"] = server["vlm_processor_path"]
    for path in (REPO_ROOT, REPO_ROOT / "XPolicyLab"):
        sys.path.insert(0, str(path))
    import setup_policy_server

    setup_policy_server.main(server)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
