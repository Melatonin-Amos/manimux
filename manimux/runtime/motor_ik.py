"""Runtime-owned, isolated IK process; no simulator or RoboDojo imports."""

import argparse
from contextlib import ExitStack
from copy import deepcopy
from numbers import Real
from multiprocessing.connection import Client, Listener
import os
from pathlib import Path
import subprocess
import signal
import tempfile
import time

import numpy as np


def validate_motor_config(config):
    robots = config.get("robots")
    if (not isinstance(robots, list) or len(robots) != 2
            or {r.get("prefix") for r in robots} != {"left", "right"}):
        raise ValueError("motor control requires left/right X5 robot metadata")
    for robot in config["robots"]:
        if (len(robot.get("joint_names", [])) != 6 or robot.get("sign") not in (1, -1)
                or len(robot.get("base_pose", [])) != 7 or not np.isfinite(robot["base_pose"]).all()
                or len(robot.get("scale", [])) != 2 or not np.isfinite(robot["scale"]).all()
                or robot["scale"][1] <= robot["scale"][0]
                or not all(isinstance(robot.get(key), str) and robot[key]
                           for key in ("robot_name", "arm", "grip", "yml_path"))):
            raise ValueError("invalid X5 motor robot metadata")
        mimic = robot.get("mimic")
        if (not isinstance(mimic, (list, tuple)) or len(mimic) != 3
                or not isinstance(mimic[0], str) or not mimic[0]
                or any(isinstance(value, bool) or not isinstance(value, Real)
                       or not np.isfinite(value) for value in mimic[1:])):
            raise ValueError("motor mimic must be [joint_name, multiplier, offset]")
    for key in ("startup_timeout_s", "solve_timeout_s"):
        value = config.get(key, 900 if key == "startup_timeout_s" else 300)
        if isinstance(value, bool) or not isinstance(value, Real) or not np.isfinite(value) or value <= 0:
            raise ValueError(f"{key} must be finite and positive")
    if not isinstance(config.get("worker_python"), str) or not config["worker_python"]:
        raise ValueError("motor control requires worker_python")
    visible = config.get("cuda_visible_devices")
    if visible is not None and (not isinstance(visible, str) or not visible.strip() or "\n" in visible):
        raise ValueError("cuda_visible_devices must be an explicit non-empty device list")
    return config


def limit_grippers(groups, measured, robots):
    result = deepcopy(groups)
    for robot in robots:
        name = robot["prefix"] + "_arm"
        limit = .2 * (robot["scale"][1] - robot["scale"][0])
        value = float(np.clip(groups[name][6], measured[name][6]-limit, measured[name][6]+limit))
        result[name][6:] = [value, value * robot["mimic"][1] + robot["mimic"][2]]
    return result


def decode_job(decoder, robots, job):
    action, measured = {}, {}
    for robot in robots:
        group = robot["prefix"] + "_arm"
        row, state = job["groups"][group], job["measured"][group]
        if len(row) != 8 or len(state) != 8 or not np.isfinite([row, state]).all():
            raise ValueError("invalid EEF target or physical motor measurement")
        action[robot["prefix"] + "_ee_pose"] = list(row[:7])
        action[robot["grip"]] = [row[7]]
        measured[robot["arm"]] = list(state[:6])
        measured[robot["grip"]] = list(state[6:])
    sequence, statuses = decoder.decode(action, measured)
    motors = []
    for control in sequence:
        groups = {}
        for robot in robots:
            name = robot["prefix"] + "_arm"
            arm = control.get(robot["arm"], {}).get("position", job["held"][name][:6])
            grip = control[robot["grip"]]["position"]
            values = np.r_[arm, grip]
            if values.shape != (8,) or not np.isfinite(values).all():
                raise ValueError("invalid decoded physical motor target")
            groups[name] = values.tolist()
        motors.append(groups)
    if len(motors) != 10:
        raise ValueError("compatible decoder must produce ten physical control targets")
    return {**deepcopy(job), "sequence": motors, "ik_status": statuses, "worker_pid": os.getpid()}


class MotorIKClient:
    def __init__(self, config, log_path):
        self.config, self.log_path = validate_motor_config(config), Path(log_path)
        self.process = self.connection = self.folder = self.listener = self.log = None
        self.pid = None

    def start(self):
        if self.folder is not None:
            raise RuntimeError("runtime IK worker already started")
        try:
            timeout = self.config.get("startup_timeout_s", 900)
            self.folder = tempfile.TemporaryDirectory(prefix="manimux-motor-ik-")
            address = str(Path(self.folder.name) / "ik.sock")
            key = os.urandom(32)
            self.listener = Listener(address, family="AF_UNIX", authkey=key)
            self.listener._listener._socket.settimeout(min(1., timeout))
            self.log_path.parent.mkdir(parents=True, exist_ok=True)
            self.log = self.log_path.open("a")
            package = str(Path(__file__).resolve().parents[2])
            env = dict(os.environ, MANIMUX_MOTOR_IK_KEY=key.hex(), MANIMUX_MOTOR_PARENT=str(os.getpid()),
                       PYTHONPATH=os.pathsep.join(filter(None, (package, os.environ.get("PYTHONPATH")))))
            if self.config.get("cuda_visible_devices") is not None:
                env["CUDA_VISIBLE_DEVICES"] = self.config["cuda_visible_devices"]
            self.process = subprocess.Popen([self.config["worker_python"], "-u", "-m",
                "manimux.runtime.motor_ik", "--socket", address],
                env=env,
                stdout=self.log, stderr=subprocess.STDOUT, start_new_session=True)
            started = time.monotonic()
            while self.connection is None:
                try:
                    self.connection = self.listener.accept()
                except TimeoutError:
                    if self.process.poll() is not None:
                        raise RuntimeError("runtime IK worker exited during startup; see motor-ik.log")
                    if time.monotonic() - started > timeout:
                        raise TimeoutError("runtime IK worker connection timed out")
            self.connection.send(self.config)
            while not self.connection.poll(.1):
                if self.process.poll() is not None:
                    raise RuntimeError("runtime IK worker exited during initialization; see motor-ik.log")
                if time.monotonic() - started > timeout:
                    raise TimeoutError("runtime IK worker initialization timed out")
            ready = self.connection.recv()
            if not ready.get("ready"):
                raise RuntimeError(f"runtime IK worker failed: {ready}")
            self.pid = ready["pid"]
        except BaseException:
            try:
                self.close()
            except Exception:
                pass
            raise

    def submit(self, job):
        self.connection.send(job)

    def poll(self):
        if self.process.poll() is not None:
            raise RuntimeError("runtime IK worker exited; see motor-ik.log")
        if not self.connection.poll():
            return None
        result = self.connection.recv()
        if "error" in result:
            raise RuntimeError("runtime IK worker: " + result["error"])
        return result

    def decode(self, job, timeout=300):
        self.submit(job)
        started = time.monotonic()
        while True:
            result = self.poll()
            if result is not None:
                return result
            if time.monotonic() - started > timeout:
                raise TimeoutError("runtime IK solve timed out")
            time.sleep(.001)

    def close(self):
        process, connection = self.process, self.connection
        folder, listener, log = self.folder, self.listener, self.log
        self.process = self.connection = self.folder = self.listener = self.log = self.pid = None
        with ExitStack() as cleanup:
            if folder is not None:
                cleanup.callback(folder.cleanup)
            if log is not None:
                cleanup.callback(log.close)
            if listener is not None:
                cleanup.callback(listener.close)
            if connection is not None:
                cleanup.callback(connection.close)
            if process is not None:
                try:
                    os.killpg(process.pid, signal.SIGTERM)
                    try:
                        process.wait(timeout=15)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait()
                except ProcessLookupError:
                    process.wait()


def main():
    import ctypes
    # The isolated solver must not retain a CUDA context after its runtime dies.
    parent = int(os.environ.pop("MANIMUX_MOTOR_PARENT"))
    if ctypes.CDLL(None).prctl(1, signal.SIGTERM, 0, 0, 0) != 0 or os.getppid() != parent:
        raise RuntimeError("runtime IK parent-death guard could not be established")
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--socket", required=True)
    args = parser.parse_args()
    key = bytes.fromhex(os.environ.pop("MANIMUX_MOTOR_IK_KEY"))
    with Client(args.socket, family="AF_UNIX", authkey=key) as connection:
        try:
            from manimux.policy_adapter.robodojo_compat import RoboDojoCompatibleDecoder
            config = validate_motor_config(connection.recv())
            decoder = RoboDojoCompatibleDecoder(config["robots"], 10)
            connection.send({"ready": True, "pid": os.getpid()})
            while True:
                job = connection.recv()
                started = time.monotonic()
                result = decode_job(decoder, config["robots"], job)
                result["decode_wall_s"] = time.monotonic() - started
                connection.send(result)
        except EOFError:
            pass
        except Exception as error:
            connection.send({"error": repr(error)})
            raise


if __name__ == "__main__":
    main()
