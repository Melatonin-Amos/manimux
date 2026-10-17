"""Launch a collection's XPolicyLab model in its existing model environment."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--collection", required=True, type=Path)
    parser.add_argument("--experiment-id")
    parser.add_argument("--local", type=Path)
    parser.add_argument(
        "--check", action="store_true", help="Resolve configuration only; no model load"
    )
    args = parser.parse_args(argv)
    from manimux.collections import select_service

    config = select_service(
        args.collection, "policy_server", local=args.local, experiment=args.experiment_id
    )
    if config["policy"]["worker"] != "xpolicylab_ws":
        raise ValueError("This launcher supports XPolicyLab deployments only")
    recipe = config["policy_server"]
    if not recipe.get("policy_name"):
        raise ValueError("policy_server.policy_name is required")
    print(
        json.dumps(
            {"policy_server": recipe, "configuration_only": args.check}, default=str, indent=2
        )
    )
    if args.check:
        return 0
    repository = Path(__file__).resolve().parents[2]
    for path in (repository, repository / "XPolicyLab"):
        sys.path.insert(0, str(path))
    if recipe["policy_name"] == "Pi_05":
        from manimux.servers.pi05 import _prepare_imports, _validate_paths

        _prepare_imports()
        _validate_paths(recipe)
    from manimux.policies.xpolicylab.server import serve

    serve(recipe)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
