#!/usr/bin/env python3
"""Serial motor bus bridge for the mos2026_2 quadruped (Unitree GO-M8010-6).

The 12 joints sit on 4 RS485/USB buses (3 motors each). We reuse the proven,
already-built ``motor_control/Linux/build/motor_ctrl <port> servo <kp> <kw>``
streaming servo: one subprocess per bus, fed rotor-position targets on stdin,
emitting per-motor feedback ("FB id=...") on stdout. We launch it with
``MOTOR_CTRL_PRINT_MS=0`` so feedback arrives every bus cycle (~hundreds of Hz)
rather than the default 5 Hz — enough to build the 50 Hz policy observation.

This module is the *only* place that knows about rotor<->joint coordinates:

    rotor_target = stand_rotor + N*dir*sim_sign*(q_sim - default)
    q_sim        = default + (rotor - stand_rotor) / (N*dir*sim_sign)
    qdot_sim     = motor_W   / (N*dir*sim_sign)
    tau_joint    = motor_T * N * dir * sim_sign           (rotor torque -> joint)

where N = gear_ratio, and dir / stand_rotor / sim_sign come from the YAML
``hardware.joints`` list (seeded from motor_control's stand_config.json).

Everything above the bridge (rl_deploy.py, the policy) works purely in the
sim/policy joint frame and never sees rotor angles.

``make_bus(cfg, dry_run=True)`` returns a SimMotorBus that mimics the motors
(first-order tracking of the commanded targets) so the full deploy node can be
exercised with no hardware attached.
"""
from __future__ import annotations

import os
import re
import select
import signal
import subprocess
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import numpy as np

# FB line emitted by motor_ctrl servo (rotor-side units):
#   FB id=3 pos=1.57320 vel=0.0123 tau=0.4500 temp=31 merr=0 ok=1 errd=0.12
_FB_RE = re.compile(
    r"FB\s+id=(\d+)\s+pos=(-?\d+(?:\.\d+)?)\s+vel=(-?\d+(?:\.\d+)?)\s+"
    r"tau=(-?\d+(?:\.\d+)?)\s+temp=(-?\d+)\s+merr=(-?\d+)\s+ok=([01])"
)
# ANGLE line emitted by motor_ctrl read (used once at startup to seed targets):
#   ANGLE id=3 ok=1 rotor=1.573 joint=0.248 deg=14.2 temp=30 err=0
_ANGLE_RE = re.compile(r"ANGLE\s+id=(\d+)\s+ok=([01])(?:.*?rotor=(-?\d+(?:\.\d+)?))?")


@dataclass
class JointMap:
    """Static per-joint wiring + sim<->rotor coordinate parameters."""
    name: str
    port: str
    motor_id: int
    dir: int            # motor-vs-joint sign, from stand_config.json
    stand_rotor: float  # rotor angle (rad) at the calibrated stand pose
    sim_sign: int       # USD/sim axis sign correction (VERIFY on hardware)
    default: float      # sim default joint angle (rad), policy frame

    @property
    def k(self) -> float:
        """Combined rotor-per-joint-radian gain (signed)."""
        return self.gear * self.dir * self.sim_sign

    gear: float = 6.33

    def q_to_rotor(self, q_sim: float) -> float:
        return self.stand_rotor + self.k * (q_sim - self.default)

    def rotor_to_q(self, rotor: float) -> float:
        return self.default + (rotor - self.stand_rotor) / self.k

    def w_to_qdot(self, w_rotor: float) -> float:
        return w_rotor / self.k

    def tau_joint(self, tau_rotor: float) -> float:
        # |joint torque| = N*|rotor torque|; sign maps through dir*sim_sign.
        return tau_rotor * self.gear * self.dir * self.sim_sign

    def tau_ff_to_rotor(self, tau_ff_joint: float) -> float:
        return tau_ff_joint / (self.gear * self.dir * self.sim_sign)


@dataclass
class JointState:
    q: np.ndarray            # sim-frame joint angle (rad)
    dq: np.ndarray           # sim-frame joint velocity (rad/s)
    tau: np.ndarray          # sim-frame joint torque estimate (N*m)
    temp: np.ndarray         # motor temperature (C)
    merr: np.ndarray         # motor error code
    ok: np.ndarray           # per-joint comms-ok flag (bool)
    stamp: float = field(default_factory=time.time)


def build_joint_maps(cfg: dict) -> list[JointMap]:
    """Construct the 12 JointMaps (in policy order) from the parsed YAML."""
    robot_key = next(k for k in cfg if k != "hardware")
    hw = cfg["hardware"]
    gear = float(hw.get("gear_ratio", 6.33))
    default = list(cfg[robot_key]["default_dof_pos"])
    joints = hw["joints"]
    if len(joints) != len(default):
        raise ValueError(f"hardware.joints has {len(joints)} entries, "
                         f"default_dof_pos has {len(default)}")
    maps: list[JointMap] = []
    for i, j in enumerate(joints):
        maps.append(JointMap(
            name=j["name"], port=j["port"], motor_id=int(j["id"]),
            dir=int(j["dir"]), stand_rotor=float(j["stand_rotor"]),
            sim_sign=int(j.get("sim_sign", 1)), default=float(default[i]),
            gear=gear,
        ))
    return maps


class MotorBus:
    """Drives the 12 real motors via one `motor_ctrl servo` process per bus."""

    def __init__(self, joint_maps: list[JointMap], motor_ctrl_bin: str,
                 motor_kp: float, motor_kw: float):
        self.maps = joint_maps
        self.n = len(joint_maps)
        self.bin = str(Path(motor_ctrl_bin).resolve())
        if not Path(self.bin).exists():
            raise FileNotFoundError(
                f"motor_ctrl binary not found: {self.bin}\n"
                f"build it: cmake --build motor_control/Linux/build --target motor_ctrl")
        self.kp = float(motor_kp)
        self.kw = float(motor_kw)

        # group joint indices by bus (preserve order)
        self.ports: list[str] = []
        self.port_joints: dict[str, list[int]] = {}
        for idx, jm in enumerate(joint_maps):
            self.port_joints.setdefault(jm.port, []).append(idx)
            if jm.port not in self.ports:
                self.ports.append(jm.port)
        self.id_to_idx = {(jm.port, jm.motor_id): i for i, jm in enumerate(joint_maps)}

        self._procs: dict[str, subprocess.Popen] = {}
        self._readers: list[threading.Thread] = []
        self._lock = threading.Lock()
        self._raw = {i: {"pos": jm.stand_rotor, "vel": 0.0, "tau": 0.0,
                         "temp": 0, "merr": 0, "ok": 0}
                     for i, jm in enumerate(joint_maps)}
        self._running = False

    # ---- lifecycle --------------------------------------------------------
    def read_initial_rotor(self, attempts: int = 1) -> np.ndarray:
        """Read each motor's current rotor angle (separate `read` processes).

        Must be called BEFORE start(): a serial port admits only one owner, and
        the servo subprocess will own it afterwards. Returns rotor angles (rad)
        indexed in policy order; falls back to stand_rotor for any motor that
        does not answer.
        """
        rotor = np.array([jm.stand_rotor for jm in self.maps], dtype=float)
        for port, idxs in self.port_joints.items():
            for idx in idxs:
                jm = self.maps[idx]
                got = None
                for _ in range(max(1, attempts)):
                    try:
                        out = subprocess.run(
                            [self.bin, port, str(jm.motor_id), "read"],
                            capture_output=True, text=True, timeout=3.0).stdout
                    except subprocess.TimeoutExpired:
                        continue
                    for line in out.splitlines():
                        m = _ANGLE_RE.search(line)
                        if m and int(m.group(1)) == jm.motor_id and m.group(2) == "1" and m.group(3):
                            got = float(m.group(3))
                            break
                    if got is not None:
                        break
                if got is not None:
                    rotor[idx] = got
                else:
                    print(f"[motor_bus] WARN: no read from {jm.name} "
                          f"(id={jm.motor_id} on {port}); using stand_rotor")
        return rotor

    def start(self, initial_q_sim: Optional[np.ndarray] = None) -> None:
        """Launch one servo per bus and (optionally) seed a hold target."""
        env = dict(os.environ, MOTOR_CTRL_PRINT_MS="0")  # per-cycle FB
        for port in self.ports:
            p = subprocess.Popen(
                [self.bin, port, "servo", f"{self.kp:.6f}", f"{self.kw:.6f}"],
                stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT, text=True, bufsize=1, env=env)
            self._procs[port] = p
            t = threading.Thread(target=self._reader, args=(port, p), daemon=True)
            t.start()
            self._readers.append(t)
        self._running = True
        if initial_q_sim is not None:
            # seed the servo so motors hold the start pose instead of snapping
            self.write_joint_targets(np.asarray(initial_q_sim, dtype=float))
        time.sleep(0.05)

    def _reader(self, port: str, proc: subprocess.Popen) -> None:
        for line in proc.stdout:
            m = _FB_RE.search(line)
            if not m:
                continue
            mid = int(m.group(1))
            idx = self.id_to_idx.get((port, mid))
            if idx is None:
                continue
            with self._lock:
                self._raw[idx] = {
                    "pos": float(m.group(2)), "vel": float(m.group(3)),
                    "tau": float(m.group(4)), "temp": int(m.group(5)),
                    "merr": int(m.group(6)), "ok": int(m.group(7)),
                }

    # ---- IO ---------------------------------------------------------------
    def write_joint_targets(self, q_sim: np.ndarray,
                            tau_ff_joint: Optional[np.ndarray] = None) -> None:
        """Send one rotor-position target line per bus (policy joint frame in)."""
        q_sim = np.asarray(q_sim, dtype=float).reshape(-1)
        tau_ff = (np.zeros(self.n) if tau_ff_joint is None
                  else np.asarray(tau_ff_joint, dtype=float).reshape(-1))
        for port, idxs in self.port_joints.items():
            parts = []
            for idx in idxs:
                jm = self.maps[idx]
                rotor = jm.q_to_rotor(q_sim[idx])
                t_rotor = jm.tau_ff_to_rotor(tau_ff[idx])
                parts.append(f"{jm.motor_id} {rotor:.6f} {t_rotor:.6f}")
            line = " ".join(parts) + "\n"
            proc = self._procs.get(port)
            if proc and proc.poll() is None and proc.stdin:
                try:
                    proc.stdin.write(line)
                    proc.stdin.flush()
                except (BrokenPipeError, ValueError):
                    pass

    def read_joint_state(self) -> JointState:
        with self._lock:
            raw = {i: dict(v) for i, v in self._raw.items()}
        q = np.empty(self.n); dq = np.empty(self.n); tau = np.empty(self.n)
        temp = np.empty(self.n); merr = np.empty(self.n); ok = np.empty(self.n, dtype=bool)
        for i, jm in enumerate(self.maps):
            r = raw[i]
            q[i] = jm.rotor_to_q(r["pos"])
            dq[i] = jm.w_to_qdot(r["vel"])
            tau[i] = jm.tau_joint(r["tau"])
            temp[i] = r["temp"]; merr[i] = r["merr"]; ok[i] = bool(r["ok"])
        return JointState(q=q, dq=dq, tau=tau, temp=temp, merr=merr, ok=ok)

    def all_ok(self) -> bool:
        st = self.read_joint_state()
        return bool(np.all(st.ok))

    def close(self) -> None:
        """Tell each servo to stop (mode=0 brake pulse) and reap the process."""
        if not self._running:
            return
        self._running = False
        for port, p in self._procs.items():
            try:
                if p.stdin:
                    p.stdin.write("stop\n"); p.stdin.flush()
            except (BrokenPipeError, ValueError, OSError):
                pass
        t0 = time.time()
        for p in self._procs.values():
            try:
                p.wait(timeout=max(0.0, 1.5 - (time.time() - t0)))
            except subprocess.TimeoutExpired:
                p.send_signal(signal.SIGTERM)
                try:
                    p.wait(timeout=1.0)
                except subprocess.TimeoutExpired:
                    p.kill()
        self._procs.clear()


class SimMotorBus(MotorBus):
    """No-hardware stand-in: first-order tracking of commanded sim targets.

    Lets rl_deploy.py run end-to-end without serial buses. It does NOT model
    contact/gravity — joints simply chase their targets — so it validates the
    control plumbing (obs build, action pipeline, FSM, conversions), not the
    physics. Use deploy/mujoco/play_mujoco.py for closed-loop dynamics.
    """

    def __init__(self, joint_maps: list[JointMap], motor_kp: float, motor_kw: float,
                 tau_const: float = 1.0):
        # skip MotorBus.__init__ (no binary / processes)
        self.maps = joint_maps
        self.n = len(joint_maps)
        self.kp = float(motor_kp)
        self.kw = float(motor_kw)
        self._q = np.array([jm.default for jm in joint_maps], dtype=float)
        self._dq = np.zeros(self.n)
        self._target = self._q.copy()
        self._tau_const = tau_const
        self._last = time.time()
        self._running = False

    def read_initial_rotor(self, attempts: int = 1) -> np.ndarray:
        return np.array([jm.q_to_rotor(jm.default) for jm in self.maps], dtype=float)

    def start(self, initial_q_sim: Optional[np.ndarray] = None) -> None:
        if initial_q_sim is not None:
            self._q = np.asarray(initial_q_sim, dtype=float).copy()
            self._target = self._q.copy()
        self._last = time.time()
        self._running = True

    def write_joint_targets(self, q_sim: np.ndarray,
                            tau_ff_joint: Optional[np.ndarray] = None) -> None:
        self._target = np.asarray(q_sim, dtype=float).reshape(-1).copy()

    def read_joint_state(self) -> JointState:
        now = time.time()
        dt = min(0.1, max(1e-4, now - self._last))
        self._last = now
        # critically-damped-ish first order chase toward target
        alpha = min(1.0, dt * 12.0)
        new_q = self._q + alpha * (self._target - self._q)
        self._dq = (new_q - self._q) / dt
        self._q = new_q
        tau = self._tau_const * (self._target - self._q) * self.kp
        return JointState(
            q=self._q.copy(), dq=self._dq.copy(), tau=tau,
            temp=np.full(self.n, 30), merr=np.zeros(self.n),
            ok=np.ones(self.n, dtype=bool))

    def close(self) -> None:
        self._running = False


def make_bus(cfg: dict, repo_root: Path, dry_run: bool = False) -> MotorBus:
    """Factory: real MotorBus, or SimMotorBus when dry_run=True."""
    maps = build_joint_maps(cfg)
    hw = cfg["hardware"]
    kp = float(hw.get("motor_kp", 8.0))
    kw = float(hw.get("motor_kw", 0.08))
    if dry_run:
        return SimMotorBus(maps, kp, kw)
    bin_path = hw.get("motor_ctrl_bin", "../../motor_control/Linux/build/motor_ctrl")
    if not Path(bin_path).is_absolute():
        bin_path = (repo_root / "deploy" / "real" / bin_path).resolve()
    return MotorBus(maps, str(bin_path), kp, kw)
