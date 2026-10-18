"""Read-only, bounded service checks; never reset, infer, poll controls or actuate."""

import json
import sys
import urllib.request
from pathlib import Path


def probe(name, plan):
    if name == "robogui":
        with urllib.request.build_opener(urllib.request.ProxyHandler({})).open(
            plan["gui_url"], timeout=1
        ) as response:
            if response.status != 200:
                raise RuntimeError(f"RoboGUI HTTP status {response.status}")
        return {"ok": True, "state": "http_responding", "detail": "Browser rendering not checked"}
    config = plan["config"]
    if name == "camera":
        from manimux.embodiments.sensor.camera_server.client import CameraClient

        from .plan import address

        _, port = address(config["camera_server"].get("rep_endpoint", "tcp://127.0.0.1:5555"))
        client = CameraClient(f"tcp://127.0.0.1:{port}", request_timeout_ms=500)
        try:
            bundle = client.get_bundle(list(config["camera_server"]["cameras"]))
            if not bundle["frames"]:
                raise RuntimeError("No camera frames received")
            return {"ok": True, "state": "fresh_frames", "detail": ", ".join(bundle["frames"])}
        finally:
            client.close()
    if name == "policy":
        from manimux.policies.capabilities import metadata_mismatches
        from manimux.policies.xpolicylab.ws_client import XPolicyLabWsClient

        client = XPolicyLabWsClient(
            url=config["policy"]["options"]["server"],
            evaluation_id="deployment-probe",
            trial_id="read-only",
            connect_timeout_s=1,
            request_timeout_s=1,
        )
        try:
            client.connect()
            expected = config["policy"].get("expected_backend")
            if expected:
                errors = metadata_mismatches(expected, client.backend_metadata)
                if errors:
                    raise RuntimeError("; ".join(errors))
            return {
                "ok": True,
                "state": "hello_verified" if expected else "hello_received",
                "detail": "Backend handshake received; no inference performed",
                "sampling_modes": sorted(client.sampling_modes),
            }
        finally:
            client.close()
    raise ValueError(f"Unknown probe {name}")


if __name__ == "__main__":
    try:
        print(json.dumps(probe(sys.argv[1], json.loads(Path(sys.argv[2]).read_text()))))
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        raise SystemExit(2) from None
