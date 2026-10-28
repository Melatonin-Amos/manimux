"""Nonblocking per-row control conversion on the existing AsyncSim runtime loop."""

from copy import deepcopy
import json
from pathlib import Path
import time

from manimux.runtime.motor_ik import MotorIKClient, limit_grippers, validate_motor_config
from manimux.types import RobotCommand


class RoboDojoMotorController:
    source_action_space = "eef_pose"

    def __init__(self, config, *, planner=None):
        robot, inference = config["robot"], config["inference"]
        if (robot["control_hz"] != 250 or config["policy"]["action_dt_s"] != .04
                or inference["algorithm"] != "serial" or inference["inference_schedule"] != "serial"
                or inference["blend_policy_steps"] or inference["handoff_skip_steps"]
                or inference["action_start_mode"] != "first_step_when_ready"
                or config["executor"]["type"] != "direct"
                or config["executor"]["motion_limits"] is not None
                or not config["run"].get("simulation_time_timeline")):
            raise ValueError("motor control requires 250Hz/25Hz, serial, unblended simulation timing")
        path = Path(robot["options"]["motor_control_config"]).expanduser().resolve()
        self.config = validate_motor_config(json.loads(path.read_text()))
        for spec in self.config["robots"]:
            model = Path(spec["yml_path"]).expanduser()
            spec["yml_path"] = str((path.parent / model).resolve() if not model.is_absolute() else model)
        self.robots = self.config["robots"]
        if robot["group_dims"] != {spec["prefix"] + "_arm": 8 for spec in self.robots}:
            raise ValueError("motor control requires physical six-joint/two-finger groups")
        self.planner = planner
        self._identity = None
        self._control_epoch = 0
        self._ik_seq = 0
        self.reset()

    def bind_identity(self, env_index, episode_id, generation, *, cuda_visible_devices=None):
        if (isinstance(env_index, bool) or not isinstance(env_index, int) or env_index < 0
                or isinstance(generation, bool) or not isinstance(generation, int) or generation < 0
                or not isinstance(episode_id, str) or not episode_id):
            raise ValueError("invalid motor control environment identity")
        self._identity = {"env_index": env_index, "episode_id": episode_id, "generation": generation}
        if cuda_visible_devices is not None:
            self.config["cuda_visible_devices"] = str(cuda_visible_devices)
            validate_motor_config(self.config)
        self.reset()

    def start(self, recording_directory=None):
        if self.planner is None:
            self.planner = MotorIKClient(
                self.config, Path(recording_directory or ".") / "motor-ik.log",
            )
        self.planner.start()

    def reset(self):
        self._control_epoch += 1
        self._pending_identity = None
        self.pending = None
        self.current = None
        self.requested = None
        self.solves = []
        self.events = []
        self.samples = []

    def sample(self, now_ns, state, source, held):
        if source is not None and source.dt_ns != 40_000_000:
            raise ValueError("motor control requires unchanged 40ms source actions")
        measured = {name: values.tolist() for name, values in state.groups.items()}
        fallback = {name: values.tolist() for name, values in held.items()}
        row = None if source is None else (now_ns - source.start_time_ns) // source.dt_ns
        active = source is not None and row is not None and 0 <= row < source.horizon_steps
        key = (source.plan_id, int(row)) if active else None
        if self.pending is not None:
            solved = self.planner.poll()
            if (solved is not None and self._identity is not None
                    and solved.get("identity") != self._pending_identity):
                self.events.append({"kind": "stale_ik_discarded", "time_ns": now_ns,
                                    "identity": solved.get("identity")})
                solved = None
            if solved is not None:
                if (solved["plan_id"], solved["source_index"]) != self.pending[:2]:
                    raise RuntimeError("IK result does not match the pending control row")
                self.solves.append(solved)
                if key == self.pending[:2]:
                    self.current = solved
                else:
                    self.events.append({"kind": "late_ik_discarded", "time_ns": now_ns,
                                        "plan_id": solved["plan_id"], "source_index": solved["source_index"]})
                self.pending = None
                self._pending_identity = None
            elif time.monotonic() - self.pending[2] > self.config.get("solve_timeout_s", 300):
                raise TimeoutError("runtime motor IK exceeded its solve deadline")
        if active and key != self.requested and self.pending is None:
            job = {"plan_id": source.plan_id, "source_index": int(row),
                   "seed_time_ns": state.monotonic_ns,
                   "groups": {name: rows[row].tolist() for name, rows in source.groups.items()},
                   "measured": deepcopy(measured), "held": deepcopy(fallback)}
            if self._identity is not None:
                self._ik_seq += 1
                self._pending_identity = {**self._identity, "control_epoch": self._control_epoch,
                                          "ik_seq": self._ik_seq}
                job["identity"] = dict(self._pending_identity)
            self.planner.submit(job)
            self.pending = (*key, time.monotonic())
            self.requested = key
        tick = None
        groups = fallback
        if active and self.current is not None and self.current["plan_id"] == source.plan_id:
            # IK does not stop physics: select by current simulation time, never replay expired ticks.
            slot = (now_ns - source.start_time_ns) // 4_000_000
            tick = min(9, max(0, int(slot - self.current["source_index"] * 10)))
            groups = self.current["sequence"][tick]
        groups = limit_grippers(groups, measured, self.robots)
        self.samples.append({"time_ns": now_ns, "plan_id": None if source is None else source.plan_id,
                             "source_index": None if self.current is None else self.current["source_index"],
                             "interpolation_tick": tick})
        return RobotCommand(groups, now_ns, None if source is None else source.plan_id)

    def check_timeout(self):
        if self.pending is not None and time.monotonic() - self.pending[2] > self.config.get("solve_timeout_s", 300):
            raise TimeoutError("runtime motor IK exceeded its solve deadline")

    def audit(self):
        return {"controller_location": "manimux_runtime", "manimux_ik": True,
                "identity": self._identity,
                "manimux_interpolation": True, "robodojo_interpolation": False,
                "control_hz_target": 250, "policy_action_hz": 25,
                "physics_during_inference": True, "physics_during_ik": True,
                "ik_seed": "current_measured_joint_state",
                "interpolation_rule": "linear_80pct_then_hold",
                "selection": "simulation_time_no_catchup",
                "solves": self.solves, "events": self.events, "samples": self.samples}

    def close(self):
        if self.planner is not None:
            self.planner.close()
