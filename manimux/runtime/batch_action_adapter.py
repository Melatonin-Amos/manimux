"""Batch RPC framing only. Environment grouping remains in AsyncSim."""

from collections.abc import Sequence

from manimux.policies.capabilities import PolicyCapabilities
from manimux.policies.xpolicylab.ws_client import CALL, XPolicyLabProtocolError, XPolicyLabWsClient


class BatchRpcClient(XPolicyLabWsClient):
    def send_call(self, method, argument):
        if not isinstance(method, str) or not method or method.startswith("_"):
            raise ValueError("batch RPC requires a public backend method")
        return self._send(CALL, {"func_name": method, "obs": argument})

    def receive_call(self, request_id):
        reply = self._await_reply(
            self._conn, request_id=request_id, message_type=CALL, timeout_s=self._request_timeout_s
        )
        payload = reply.get("payload")
        if not isinstance(payload, dict) or payload.get("ok") is not True:
            raise XPolicyLabProtocolError("batch CALL did not return a successful payload")
        return payload.get("result")


class BatchActionAdapter:
    """A dedicated connection owns each update_obs_batch/get_action_batch pair.

    The current public get_action_batch RPC accepts only an env-index list.
    It has no per-member sampling argument; non-default sampling is rejected,
    never stripped or simulated with multiple single-environment calls.
    """

    def __init__(self, client, *, encode_request, decode_action):
        self.client = client
        self.encode_request = encode_request
        self.decode_action = decode_action
        self._inflight = False

    @property
    def capabilities(self):
        return PolicyCapabilities(
            sampling_modes=frozenset({"default"}),
            multi_inflight=False,
            backend_metadata=dict(self.client.backend_metadata),
        )

    def start(self):
        self.client.connect()
        self.client.reset()

    def infer(self, members, on_forwarded):
        if self._inflight:
            raise RuntimeError("batch transaction is already active")
        members = tuple(members)
        indices = [identity.env_index for identity, _ in members]
        if not members or len(set(indices)) != len(indices):
            raise ValueError("batch must contain distinct environments")
        observations = []
        for identity, request in members:
            observation, sampling = self.encode_request(identity, request)
            if sampling != {"mode": "default"}:
                raise ValueError(
                    "get_action_batch RPC does not support per-member sampling conditions"
                )
            observations.append({**observation, "env_idx": identity.env_index})
        self._inflight = True
        try:
            self.client.call("update_obs_batch", obs=observations)
            request_id = self.client.send_call("get_action_batch", indices)
            # _send returns only after the complete frame was handed to the
            # transport. Local queue admission is deliberately not an ACK.
            on_forwarded()
            results = self.client.receive_call(request_id)
            if (
                not isinstance(results, Sequence)
                or isinstance(results, (str, bytes))
                or len(results) != len(members)
            ):
                raise XPolicyLabProtocolError("batch response count does not match request members")
            return [
                self.decode_action(identity, request, action)
                for (identity, request), action in zip(members, results, strict=True)
            ]
        finally:
            self._inflight = False

    def reset(self):
        if self._inflight:
            raise RuntimeError("reset cannot interleave with a batch transaction")
        self.client.reset()

    def close(self):
        self.client.close()
