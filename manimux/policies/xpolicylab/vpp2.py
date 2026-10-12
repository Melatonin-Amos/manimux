"""Thin VPP2 per-client CALL sequence over the existing XPolicyLab worker."""

from manimux.policies.xpolicylab.client import XPolicyLabWsPolicyModel
from manimux.policies.xpolicylab.codec import _layouts_from_options, encode_observation
from manimux.policy_adapter.vpp2.arx_x5 import REPLAN_STEPS, VPP2Request, VPP2Snapshot

CLIENT_ID_FIELD = "_vpp2_client_id"


class VPP2WsPolicyModel(XPolicyLabWsPolicyModel):
    def reset(self, session_id):
        self._failed = True
        self._inference_count = 0
        super().reset(session_id)
        try:
            self._client.call("reset_client", obs={CLIENT_ID_FIELD: session_id})
        except Exception:
            self.close()
            raise
        self._failed = False

    def _encode(self, snapshot, instruction):
        if not isinstance(snapshot, VPP2Snapshot):
            raise TypeError("VPP2 requires state mapped from visible feedback")
        observation = encode_observation(
            snapshot, layouts=self._layouts, camera_map=self._camera_map,
            instruction=instruction, frequency=self._frequency,
        )
        env_index = self._layout_options.get("env_index", 0)
        observation["state"].update(snapshot.model_state)
        observation["env_idx"] = env_index
        observation[CLIENT_ID_FIELD] = self._session_id
        observation["additional_info"]["sensor_sources"] = {
            "robot_state": {"sequence": snapshot.state.sequence, "capture_monotonic_ns": snapshot.state.monotonic_ns},
            "cameras": {name: {"sequence": frame.sequence, "capture_monotonic_ns": frame.capture_monotonic_ns}
                        for name, frame in snapshot.frames.items()},
        }
        return observation

    def _infer_wire(self, request):
        if self._failed or request.session_id != self._session_id or self._client is None:
            raise RuntimeError("VPP2 session requires reset after an incomplete stateful call")
        if not isinstance(request, VPP2Request):
            raise TypeError("VPP2 requires an explicitly transported observation history")
        expected = 1 if self._inference_count == 0 else REPLAN_STEPS
        if len(request.observation_history) != expected:
            raise ValueError(f"VPP2 requires {expected} new native observations")
        if not self._layouts:
            self._layouts = _layouts_from_options(
                self._layout_options, {name: values.size for name, values in request.observation.state.groups.items()},
            )
        history = [self._encode(snapshot, request.instruction) for snapshot in request.observation_history]
        try:
            # Keep plain RGB arrays in nested history: the upstream image codec
            # decodes top-level vision only. Preprocessing stays in VPP2.
            self._client.call("update_obs", obs={
                CLIENT_ID_FIELD: self._session_id,
                "env_idx": history[-1]["env_idx"],
                "_vpp2_observation_history": history,
            })
            result = self._client.call("get_action", obs={CLIENT_ID_FIELD: self._session_id})
        except Exception:
            self._failed = True
            raise
        self._inference_count += 1
        return result


def build_model(config):
    return VPP2WsPolicyModel(config)
