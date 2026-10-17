"""Check a resolved experiment catalog without starting any services or hardware."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from manimux.experiment_list import load_experiment_list


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config-list", required=True, type=Path)
    parser.add_argument("--ratio", type=float, default=32 / 50)
    args = parser.parse_args()
    catalog = load_experiment_list(args.config_list)
    # A managed study declares its storage and setting contract next to its folders.
    protocol_path = catalog.path.parent.parent / "protocol.yaml"
    protocol = yaml.safe_load(protocol_path.read_text()) if protocol_path.is_file() else None
    repository = Path(__file__).resolve().parents[2]
    for choice in catalog.choices.values():
        config = choice.config
        inference = config["inference"]
        algorithm = inference["algorithm"]
        horizon = config["policy"]["horizon_policy_steps"]
        expected = round(horizon * args.ratio)
        if algorithm == "autohorizon":
            actual = None
        elif algorithm == "rtc":
            actual = inference["rtc"]["min_execute_policy_steps"]
        elif algorithm == "paint":
            actual = inference["paint"]["execution_policy_steps"]
        elif algorithm == "bid_backward":
            actual = inference["bid"]["replan_policy_steps"]
        elif algorithm == "serial":
            actual = inference["max_chunk_policy_steps"]
        elif algorithm == "async":
            if inference["request_trigger"] != "progress" or inference["max_chunk_policy_steps"] is not None:
                raise ValueError(f"{choice.id}: Async must use progress and retain the full horizon")
            actual = inference["chunk_policy_steps"]
        else:
            raise ValueError(f"{choice.id}: unsupported audit method {algorithm}")
        if actual is not None and actual != expected:
            raise ValueError(f"{choice.id}: expected K={expected}, got {actual}")
        if config["executor"]["type"] != "direct" or inference["blend_policy_steps"] != 0:
            raise ValueError(f"{choice.id}: expected Direct and blend=0")
        if protocol is not None:
            relative = choice.path.relative_to(protocol_path.parent).with_suffix("")
            expected_output = repository / protocol["recording_root"] / relative
            pose = config["run"].get("initial_pose")
            if pose is not None:
                expected_output = expected_output / "initial_pose" / pose["id"]
            if Path(config["run"]["output_dir"]).resolve() != expected_output.resolve():
                raise ValueError(f"{choice.id}: config and recording directories do not match")
            if horizon != protocol["horizon_policy_steps"]:
                raise ValueError(f"{choice.id}: horizon differs from the study protocol")
            if config["policy"]["action_dt_s"] != protocol["action_dt_s"]:
                raise ValueError(f"{choice.id}: action interval differs from the study protocol")
            if config["policy_server"]["checkpoint_variant"] != protocol["checkpoint_variant"]:
                raise ValueError(f"{choice.id}: checkpoint differs from the study protocol")
            if algorithm == "rtc" and inference["rtc"]["beta"] != protocol["rtc_beta"]:
                raise ValueError(f"{choice.id}: RTC beta differs from the study protocol")
        print(f"{choice.id}: H={horizon}, K={actual if actual is not None else 'adaptive'}, trigger={inference['request_trigger']}")
    print(f"PASS: {len(catalog.choices)} resolved choices; configuration checks only.")
    print("Async/RTC/PAINT source progress includes latency-trimmed rows; K is not a physical executed-row count.")


if __name__ == "__main__":
    main()
