#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""GO-M8010-6 电机管理 —— 网页版。

tkinter 版 (motor_id_gui.py) 在部分 Linux 上中文字体会显示成方块（豆腐块），
原因是 Tk 默认字体不含 CJK 字形。本网页版改由浏览器渲染界面，中文显示正常，
功能与 tkinter 版一致：查看 / 修改电机 ID、切回电机模式、扫描全部串口、
驱动 / 停止、读取所有电机角度。

实现说明：
  - 纯标准库（http.server），无第三方依赖。
  - 默认只监听 127.0.0.1（本机），避免把电机控制暴露到局域网。
    如确需远程访问：HOST=0.0.0.0 PORT=8000 python3 scripts/motor_web.py
  - 底层仍调用 swboot / changeid / swmotor / motor_ctrl，逻辑与 GUI 版对齐。

运行：
    python3 scripts/motor_web.py
    # 然后浏览器打开 http://127.0.0.1:8000
"""

import json
import math
import os
import re
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

# --- 路径与常量（与 motor_id_gui.py 保持一致） ---
_DEFAULT_SDK_ROOT = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "Linux")
)
SDK_ROOT = os.environ.get("UNITREE_MOTOR_SDK", _DEFAULT_SDK_ROOT)
TOOL_DIR = os.path.join(
    SDK_ROOT, "motor_tools", "Unitree_MotorTools_v0.2.0_x86_64_Linux"
)
BUILD_DIR = os.path.join(SDK_ROOT, "build")
MOTOR_CTRL = os.path.join(BUILD_DIR, "motor_ctrl")
SWBOOT = os.path.join(TOOL_DIR, "swboot")
CHANGEID = os.path.join(TOOL_DIR, "changeid")
SWMOTOR = os.path.join(TOOL_DIR, "swmotor")

SUDO_PASSWORD = "1"
DEFAULT_SPEED = 0.1
# 本机共 12 个电机，从 1 开始编号，合法单播 ID 为 1~12（手册 ID 15 为广播地址，单播无返回）。
MIN_MOTOR_ID = 1
MAX_MOTOR_ID = 12
NEED_SUDO = os.geteuid() != 0
MAX_LOG = 4000  # 日志缓冲最多保留的行数

# GO-M8010-6 减速比：转子角 = 关节角 * 6.33（与 motor_ctrl.cpp 一致）
GEAR_RATIO = 6.33
# --- 位置控制（平滑位置环，与 example/main.cpp 的位置控制一致）---
# 调用 motor_ctrl <port> <id> moveto <角度°> <关节速度> <kp> <kw>：先读当前位置当
# 起点，目标位置按速度平滑递增（轨迹），K_P 跟位置 + K_W 阻尼 + 速度前馈，
# 到位后保持；点「停止」时进程收到 SIGTERM，退出前自动松力。
# 低速也丝滑：力矩主要来自 K_P×位置误差（编码器位置干净），而非低速时又糙又跳的速度估计。
POS_MAX_DEG = 90.0       # 单次相对转动最大角度（关节°），防止误填过大
POS_VEL_DEFAULT = 0.5     # 关节角速度默认 (rad/s)，越小越慢越稳
POS_VEL_MAX = 5.0         # 关节角速度上限 (rad/s)，防止误填过快
POS_KP_DEFAULT = 2.0      # 位置环刚度，越大越"硬"跟得越紧（抖就调小）
POS_KP_MAX = 25.0         # K_P 上限（手册 0~25.599）
POS_KW_DEFAULT = 0.1      # 速度阻尼，抑制抖动（嗡鸣就调小）
POS_KW_MAX = 25.0         # K_W 上限（手册 0~25.599）
POS_T_DEFAULT = 0.0       # 力矩前馈（转子 N·m，可正负），常驻施加，补偿重力/负载
POS_T_MAX = 3.0           # 力矩前馈幅值上限（防止误填过大力矩损坏电机/越限位）

# --- 姿态标定 / 配置文件 ---
CONFIG_DIR = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "config")
)
JOINT_MAP_PATH = os.path.join(CONFIG_DIR, "joint_map.default.json")
STAND_CONFIG_PATH = os.path.join(CONFIG_DIR, "stand_config.json")
# 判定转动方向的死区：关节角变化小于此值(rad)视为「未动/方向不明」(0)
DIR_DEADZONE_RAD = 0.01


def load_joint_map():
    """读取关节映射模板（robot / gear_ratio / joints / execute）。读不到返回空 dict。"""
    try:
        with open(JOINT_MAP_PATH, encoding="utf-8") as f:
            return json.load(f)
    except (OSError, ValueError):
        return {}


# ----------------------------------------------------------------- 工具函数
def list_serial_ports():
    # ttyUSB 排在最前：电机走 USB-RS485，默认应优先选 ttyUSB，
    # 否则下拉框会默认选到 ttyACM/ttyS，导致「读取所有角度」读错串口。
    usb, acm, other = [], [], []
    for name in os.listdir("/dev"):
        if name.startswith("ttyUSB"):
            usb.append("/dev/" + name)
        elif name.startswith("ttyACM"):
            acm.append("/dev/" + name)
        elif name.startswith("ttyS"):
            other.append("/dev/" + name)
    ports = sorted(usb) + sorted(acm) + sorted(other)
    if "/dev/ttyUSB0" not in ports:
        ports.insert(0, "/dev/ttyUSB0")
    return ports


def list_ttyusb_ports():
    return sorted("/dev/" + n for n in os.listdir("/dev") if n.startswith("ttyUSB"))


def parse_ids(text):
    """从 swboot 输出里解析电机 ID（兼容多种格式），与 GUI 版一致。"""
    patterns = [
        r"motor\s*[\[\(_]?\s*(\d+)",
        r"\bID\s*[:=]?\s*(\d+)",
        r"\bid\s*[:=_]?\s*(\d+)",
    ]
    ids = []
    for pat in patterns:
        for m in re.finditer(pat, text, re.IGNORECASE):
            try:
                n = int(m.group(1))
            except ValueError:
                continue
            if 0 <= n <= 15:
                ids.append(n)
    seen = set()
    uniq = []
    for i in ids:
        if i not in seen:
            seen.add(i)
            uniq.append(i)
    return uniq


def parse_angle_line(line):
    """解析 motor_ctrl read 的输出：
        ANGLE id=0 ok=1 rotor=1.234 joint=0.195 deg=11.17 temp=30 err=0
        ANGLE id=1 ok=0
    """
    m = re.match(r"\s*ANGLE\s+id=(\d+)\s+ok=([01])(.*)", line)
    if not m:
        return None
    mid = int(m.group(1))
    ok = m.group(2) == "1"
    rest = m.group(3)

    def grab(key):
        mm = re.search(rf"{key}=(-?\d+(?:\.\d+)?)", rest)
        return mm.group(1) if mm else ""

    if ok:
        return {
            "id": mid, "ok": True,
            "rotor": grab("rotor"), "joint": grab("joint"),
            "deg": grab("deg"), "temp": grab("temp"), "err": grab("err"),
        }
    return {"id": mid, "ok": False}


# ----------------------------------------------------------------- 控制器
class Controller:
    """承载全部串口操作；每个命令在后台线程执行，输出写入共享日志缓冲。

    前端通过轮询 /api/state 拿到：忙闲状态、是否驱动中、检测到的 ID、
    最近一次读到的角度，以及新增的日志行。
    """

    def __init__(self):
        self.lock = threading.RLock()
        self.log = []
        self.dropped = 0          # 已被丢弃（截断）的日志行数，用于 since 偏移
        self.busy = False         # 前台短命令是否在跑
        self.cmd_proc = None      # 当前前台子进程（供取消）
        self.drive_proc = None    # 持续运行的驱动子进程（速度驱动 / 位置计时驱动共用）
        self.drive_port = ""      # 当前驱动的串口（供急停补发停止脉冲）
        self.drive_id = None      # 当前驱动的电机 id（供急停补发停止脉冲）
        self.cancel = threading.Event()
        self.detected_ids = []
        self.angles = []
        self.active_port = ""     # 最近一次扫描到电机响应的串口，供前端自动选中
        self.status = "就绪"
        # -- 姿态标定 --
        self.joint_map = load_joint_map()
        self.pose_prone = {}      # {id(int): 关节角 rad} 趴姿
        self.pose_stand = {}      # {id(int): 关节角 rad} 站姿
        self.continuous = False   # 持续读取模式是否开启
        self.cont_stop = threading.Event()
        self.cont_proc = None     # 持续读取当前的子进程（供停止时终止）
        # 尽力而为地记录电机是否处于工厂(boot)模式：
        # 修改 ID(changeid) 会切到工厂模式 -> True；
        # 切回电机模式(swmotor) -> False。仅反映本进程内的操作，
        # 不能感知电机重新上电后仍停留在工厂模式的情况。
        self.factory_mode = False

    # -- 共享状态 --
    def append(self, line):
        with self.lock:
            self.log.append(line)
            over = len(self.log) - MAX_LOG
            if over > 0:
                del self.log[:over]
                self.dropped += over

    def set_status(self, s):
        with self.lock:
            self.status = s

    def snapshot(self, since):
        with self.lock:
            total = self.dropped + len(self.log)
            start = max(since, self.dropped)
            lines = self.log[start - self.dropped:]
            driving = self.drive_proc is not None and self.drive_proc.poll() is None
            return {
                "busy": self.busy,
                "driving": driving,
                "status": self.status,
                "detected_ids": self.detected_ids,
                "angles": self.angles,
                "active_port": self.active_port,
                "factory_mode": self.factory_mode,
                "continuous": self.continuous,
                "pose_prone": self.pose_prone,
                "pose_stand": self.pose_stand,
                "log": lines,
                "log_index": total,
            }

    # -- 子进程 --
    def _spawn(self, cmd):
        full = (["sudo", "-S", "-p", ""] + cmd) if NEED_SUDO else cmd
        proc = subprocess.Popen(
            full, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT, text=True, bufsize=1,
        )
        if NEED_SUDO:
            try:
                proc.stdin.write(SUDO_PASSWORD + "\n")
                proc.stdin.flush()
            except (BrokenPipeError, OSError):
                pass
        try:
            proc.stdin.close()
        except (BrokenPipeError, OSError):
            pass
        return proc

    # -- 前台单命令（查看 ID / 改 ID / 切电机模式 / 读角度） --
    def _foreground(self, cmd, title, post_parse_ids=False, parse_angles=False,
                    factory_after=None):
        self.append("\n" + "=" * 60)
        shown = (["sudo"] + cmd) if NEED_SUDO else cmd
        self.append(f"[{title}]  $ {' '.join(shown)}")
        self.set_status(f"运行中: {title}")
        buf = []
        rows = []
        try:
            proc = self._spawn(cmd)
        except FileNotFoundError as e:
            self.append(f"[错误] {e}")
            self._finish(title)
            return
        with self.lock:
            self.cmd_proc = proc
        for line in proc.stdout:
            line = line.rstrip("\n")
            buf.append(line)
            self.append(line)
            if parse_angles:
                r = parse_angle_line(line)
                if r:
                    rows.append(r)
        proc.wait()
        with self.lock:
            self.cmd_proc = None
        self.append(f"[退出码 {proc.returncode}]")

        if post_parse_ids:
            ids = parse_ids("\n".join(buf))
            if ids:
                self.append(f"==> 检测到电机 ID: {', '.join(map(str, ids))}")
                with self.lock:
                    self.detected_ids = ids
            else:
                self.append("==> 未解析到电机 ID（请查看上方原始输出）")

        if parse_angles:
            responders = [r["id"] for r in rows if r.get("ok")]
            with self.lock:
                self.angles = [r for r in rows if r.get("ok")]
                if responders:
                    self.detected_ids = responders
            if responders:
                self.append(f"==> 响应电机 ID: {', '.join(map(str, responders))}（角度见上方表格）")
            elif self.factory_mode:
                self.append("==> 无电机响应：电机疑似仍处于工厂(boot)模式"
                            "（之前做过「修改 ID」）。请先点「切回电机模式 (swmotor)」再读取角度。")
            else:
                self.append("==> 无电机响应（确认已处于电机模式且接线正常）")

        if factory_after is not None:
            with self.lock:
                self.factory_mode = factory_after

        self._finish(title)

    # -- 多串口批量（扫描全部 / 切回全部） --
    def _multi(self, tool, ports, title, post_parse_ids, factory_after=None):
        self.append("\n" + "=" * 60)
        self.append(f"[{title}] 串口列表: {', '.join(ports)}")
        self.set_status(f"{title} 进行中（可取消）")
        per = {}
        for port in ports:
            if self.cancel.is_set():
                self.append("[已取消] 跳过剩余串口")
                break
            cmd = [tool, port]
            shown = (["sudo"] + cmd) if NEED_SUDO else cmd
            self.append("\n--- " + port + " ---")
            self.append("$ " + " ".join(shown))
            try:
                proc = self._spawn(cmd)
            except FileNotFoundError as e:
                self.append(f"[错误] {e}")
                continue
            with self.lock:
                self.cmd_proc = proc
            buf = []
            for line in proc.stdout:
                line = line.rstrip("\n")
                buf.append(line)
                self.append(line)
            proc.wait()
            with self.lock:
                self.cmd_proc = None
            self.append(f"[{port} 退出码 {proc.returncode}]")
            if post_parse_ids:
                per[port] = parse_ids("\n".join(buf))

        if post_parse_ids:
            self.append("\n===== 扫描汇总 =====")
            all_ids = []
            for p in ports:
                ids = per.get(p, [])
                if ids:
                    self.append(f"{p}: ID = {', '.join(map(str, ids))}")
                    for i in ids:
                        if i not in all_ids:
                            all_ids.append(i)
                else:
                    self.append(f"{p}: (未检测到 / 已取消)")
            if all_ids:
                with self.lock:
                    self.detected_ids = all_ids

        if factory_after is not None:
            with self.lock:
                self.factory_mode = factory_after

        self._finish(title)

    # -- 无损扫描：遍历全部串口用 motor_ctrl read 列出有响应的 ID --
    # 与 swboot 扫描不同，这里发零力矩运控帧，电机保持电机模式、不会被切到工厂模式。
    def _scan_read(self, ports, title):
        self.append("\n" + "=" * 60)
        self.append(f"[{title}] 串口列表: {', '.join(ports)}")
        self.set_status(f"{title} 进行中（可取消）")
        per = {}
        per_rc = {}
        all_rows = []
        for port in ports:
            if self.cancel.is_set():
                self.append("[已取消] 跳过剩余串口")
                break
            cmd = [MOTOR_CTRL, port, "all", "read"]
            shown = (["sudo"] + cmd) if NEED_SUDO else cmd
            self.append("\n--- " + port + " ---")
            self.append("$ " + " ".join(shown))
            try:
                proc = self._spawn(cmd)
            except FileNotFoundError as e:
                self.append(f"[错误] {e}")
                continue
            with self.lock:
                self.cmd_proc = proc
            rows = []
            for line in proc.stdout:
                line = line.rstrip("\n")
                self.append(line)
                r = parse_angle_line(line)
                if r and r.get("ok"):
                    rows.append(r)
            proc.wait()
            with self.lock:
                self.cmd_proc = None
            self.append(f"[{port} 退出码 {proc.returncode}]")
            per[port] = [r["id"] for r in rows]
            per_rc[port] = proc.returncode
            all_rows.extend(rows)

        self.append("\n===== 无损扫描汇总（保持电机模式）=====")
        all_ids = []
        for p in ports:
            if p not in per:
                self.append(f"{p}: (已取消，未扫描)")
                continue
            ids = per[p]
            if ids:
                self.append(f"{p}: 响应 ID = {', '.join(map(str, ids))}")
                for i in ids:
                    if i not in all_ids:
                        all_ids.append(i)
            elif per_rc.get(p, 0) != 0:
                self.append(f"{p}: 串口打开/通信失败（退出码 {per_rc[p]}），"
                            "检查设备权限或该口是否被占用")
            else:
                self.append(f"{p}: 无响应（该串口上没有电机应答——未接电机/未上电/485 接线问题）")
        good_port = next((p for p in ports if per.get(p)), "")
        with self.lock:
            self.angles = all_rows
            if all_ids:
                self.detected_ids = all_ids
                # 有电机用运控帧应答 => 它们处于电机模式，清掉工厂模式标志
                self.factory_mode = False
            if good_port:
                # 自动选中真正接电机的串口，方便接着用「读取所有角度」/驱动
                self.active_port = good_port
        if good_port:
            self.append(f"==> 电机所在串口: {good_port}（已自动选中）")
        if all_ids:
            self.append(f"==> 共响应电机 ID: {', '.join(map(str, all_ids))}（未切换模式）")
        else:
            self.append("==> 无电机响应：确认接线与供电；若之前改过 ID，电机可能仍在工厂模式"
                        "（不应答运控指令），请先点「切回电机模式 (swmotor)」再扫描。")
        self._finish(title)

    # -- 持续读取：循环跑 motor_ctrl read 刷新 self.angles，用于实时观察转动方向 --
    # 不往日志刷每行（避免刷屏），只静默更新角度；start/stop 各记一条日志。
    def _cont_start(self):
        with self.lock:
            if self.continuous:
                return
            self.continuous = True
        self.cont_stop.clear()
        self.append("\n" + "=" * 60)
        self.append("[持续读取] 开始循环读取角度（约 1~2Hz）；记录趴/站姿、观察实时方向；再点一次停止。")
        self.set_status("持续读取中…")
        threading.Thread(target=self._cont_loop, daemon=True).start()

    def _cont_loop(self):
        ports = list_ttyusb_ports()
        while not self.cont_stop.is_set():
            rows = []
            for port in ports:
                if self.cont_stop.is_set():
                    break
                try:
                    proc = self._spawn([MOTOR_CTRL, port, "all", "read"])
                except FileNotFoundError:
                    continue
                with self.lock:
                    self.cont_proc = proc
                for line in proc.stdout:
                    r = parse_angle_line(line.rstrip("\n"))
                    if r and r.get("ok"):
                        rows.append(r)
                proc.wait()
                with self.lock:
                    self.cont_proc = None
            with self.lock:
                if rows:
                    self.angles = rows
                    self.detected_ids = [r["id"] for r in rows]
            if self.cont_stop.wait(0.2):
                break
        with self.lock:
            self.continuous = False
            self.cont_proc = None
        self.append("[持续读取] 已停止。")
        self.set_status("持续读取已停止")

    def _cont_stop_now(self):
        self.cont_stop.set()
        with self.lock:
            proc = self.cont_proc
        if proc is not None and proc.poll() is None:
            try:
                proc.terminate()
            except Exception:
                pass

    # -- 姿态标定：把当前角度快照成趴姿 / 站姿 --
    def _capture_pose(self, which):
        with self.lock:
            rows = list(self.angles)
        pose = {}
        for r in rows:
            if not r.get("ok"):
                continue
            try:
                pose[int(r["id"])] = float(r["joint"])
            except (KeyError, ValueError, TypeError):
                continue
        label = "趴姿" if which == "prone" else "站姿"
        if not pose:
            self.append(f"[标定] 当前无可用角度，无法记录{label}；"
                        "请先「读取所有角度」或开启「持续读取」。")
            return False, f"当前无可用角度，无法记录{label}"
        with self.lock:
            if which == "prone":
                self.pose_prone = pose
            else:
                self.pose_stand = pose
        ids = ", ".join(str(i) for i in sorted(pose))
        self.append(f"[标定] 已记录{label}: {len(pose)} 个关节（id {ids}）")
        return True, "ok"

    # -- 保存 stand_config.json（沿用 joint_map 模板，补 prone/stand/dir） --
    def _save_config(self):
        jm = self.joint_map or {}
        with self.lock:
            prone = dict(self.pose_prone)
            stand = dict(self.pose_stand)
        if not prone and not stand:
            return False, "尚未记录任何姿态，先记录趴姿/站姿再保存"

        gear = jm.get("gear_ratio") or 6.33

        def deg(x):
            return round(math.degrees(x), 3) if x is not None else None

        def rotor(x):
            return round(x * gear, 6) if x is not None else None

        out_joints = []
        for j in jm.get("joints", []):
            jid = j.get("id")
            p = prone.get(jid)
            s = stand.get(jid)
            d = None
            delta_rotor = None
            if p is not None and s is not None:
                diff = s - p
                d = 0 if abs(diff) < DIR_DEADZONE_RAD else (1 if diff > 0 else -1)
                delta_rotor = round(diff * gear, 6)
            out_joints.append({
                "name": j.get("name"), "port": j.get("port"), "id": jid,
                # robot_web.py 执行/验证用的字段（转子角坐标，delta=站-趴）
                "prone_rotor": rotor(p), "stand_rotor": rotor(s),
                "delta_rotor": delta_rotor,
                # 关节角坐标 + 方向，便于人看
                "prone_rad": round(p, 6) if p is not None else None,
                "stand_rad": round(s, 6) if s is not None else None,
                "prone_deg": deg(p), "stand_deg": deg(s),
                "dir": d,
            })
        config = {
            "robot": jm.get("robot"),
            "gear_ratio": gear,
            "_comment": ("由 motor_web.py 姿态标定生成。prone_rotor/stand_rotor/delta_rotor 为转子角(rad)，"
                         "供 robot_web.py 站立执行/验证；prone_rad/stand_rad/dir 为关节角与方向，便于人看。"),
            "_generated": time.strftime("%Y-%m-%d %H:%M:%S"),
            "joints": out_joints,
            "execute": jm.get("execute", {}),
        }
        try:
            os.makedirs(CONFIG_DIR, exist_ok=True)
            with open(STAND_CONFIG_PATH, "w", encoding="utf-8") as f:
                json.dump(config, f, ensure_ascii=False, indent=2)
        except OSError as e:
            self.append(f"[标定] 保存失败: {e}")
            return False, f"保存失败: {e}"
        n_full = sum(1 for j in out_joints if j["dir"] is not None)
        self.append(f"[标定] 已保存配置: {STAND_CONFIG_PATH}"
                    f"（{n_full}/{len(out_joints)} 个关节含完整 趴+站 方向）")
        return True, STAND_CONFIG_PATH

    # -- 读取磁盘上已保存的 stand_config.json（供前端只读展示） --
    def read_saved_config(self):
        if not os.path.isfile(STAND_CONFIG_PATH):
            return {"exists": False, "path": STAND_CONFIG_PATH}
        try:
            with open(STAND_CONFIG_PATH, encoding="utf-8") as f:
                cfg = json.load(f)
        except (OSError, ValueError) as e:
            return {"exists": True, "path": STAND_CONFIG_PATH, "error": str(e)}
        try:
            mtime = time.strftime("%Y-%m-%d %H:%M:%S",
                                  time.localtime(os.path.getmtime(STAND_CONFIG_PATH)))
        except OSError:
            mtime = ""
        return {"exists": True, "path": STAND_CONFIG_PATH, "mtime": mtime, "config": cfg}

    # -- 驱动 / 停止 --
    def _drive(self, port, mid, speed):
        # 完全沿用 main.cpp：mode=1, K_P=0, K_W=0.05, T=0，只设速度 W=speed。
        # speed 为电机【转子】转速(rad/s)，直接写入 cmd.W；输出端转速 = speed/6.33。
        # duration=0 表示一直转到点「停止」。K_W=0.05 偏小，低速力矩小转不动时调大转速。
        cmd = [MOTOR_CTRL, port, str(mid), "drive", f"{speed}", "0"]
        shown = (["sudo"] + cmd) if NEED_SUDO else cmd
        self.append("\n" + "=" * 60)
        self.append(f"[驱动 id={mid} 转子转速={speed} rad/s]  $ {' '.join(shown)}")
        self.set_status(f"驱动中: id={mid} speed={speed} rad/s")
        try:
            proc = self._spawn(cmd)
        except FileNotFoundError as e:
            self.append(f"[错误] {e}")
            self.set_status("驱动已停止")
            return
        with self.lock:
            self.drive_proc = proc
            self.drive_port = port
            self.drive_id = mid
        threading.Thread(target=self._drive_reader, args=(proc,), daemon=True).start()

    def _drive_reader(self, proc):
        for line in proc.stdout:
            self.append(line.rstrip("\n"))
        proc.wait()
        self.append(f"[驱动进程退出码 {proc.returncode}]")
        with self.lock:
            self.drive_proc = None
        self.set_status("驱动已停止")

    # -- 位置控制：平滑转动指定角度（调用 motor_ctrl moveto，逻辑同 main.cpp）--
    def _position(self, port, mid, deg, vel, kp, kw, t_ff, on_arrive):
        """从当前位置平滑转动 deg(关节角，正/负=正/反转)，关节速度 vel(rad/s)。

        交给 motor_ctrl moveto：读当前位置当起点→目标按速度平滑递增→K_P 跟位置 +
        K_W 阻尼 + 速度前馈 + 力矩前馈 t_ff。on_arrive 控制到位后行为：
          release=松力退出 / hold=持续保持(点停止才退) / keep=保留命令退出不松力。
        长驻进程，按 drive_proc 跟踪，复用「停止」逻辑（停止一律松力）。
        """
        cmd = [MOTOR_CTRL, port, str(mid), "moveto",
               f"{deg}", f"{vel}", f"{kp}", f"{kw}", f"{t_ff}", on_arrive]
        shown = (["sudo"] + cmd) if NEED_SUDO else cmd
        self.append("\n" + "=" * 60)
        self.append(f"[位置控制 id={mid} 角度={deg:+.1f}°(关节) 速度={vel} rad/s "
                    f"K_P={kp} K_W={kw} T前馈={t_ff} 到位={on_arrive}]  $ {' '.join(shown)}")
        self.set_status(f"位置控制: id={mid} 平滑转动 {deg:+.1f}°（到位={on_arrive}）")
        try:
            proc = self._spawn(cmd)
        except FileNotFoundError as e:
            self.append(f"[错误] {e}")
            self.set_status("位置控制失败")
            return
        with self.lock:
            self.drive_proc = proc
            self.drive_port = port
            self.drive_id = mid
        threading.Thread(target=self._drive_reader, args=(proc,), daemon=True).start()

    def _stop(self, port, mid):
        with self.lock:
            proc = self.drive_proc
        if proc is not None and proc.poll() is None:
            self.append("[停止] 终止驱动子进程 ...")
            try:
                proc.terminate()
                try:
                    proc.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    proc.kill()
            except Exception as e:
                self.append(f"[警告] 终止驱动失败: {e}")
        # 再补一段 mode=0 停止脉冲，确保电机真正停下
        cmd = [MOTOR_CTRL, port, str(mid), "stop", "500"]
        shown = (["sudo"] + cmd) if NEED_SUDO else cmd
        self.append(f"[停止脉冲]  $ {' '.join(shown)}")
        try:
            p = self._spawn(cmd)
            for line in p.stdout:
                self.append(line.rstrip("\n"))
            p.wait()
            self.append(f"[停止脉冲退出码 {p.returncode}]")
        except FileNotFoundError as e:
            self.append(f"[错误] {e}")
        with self.lock:
            self.drive_proc = None
        self.set_status("驱动已停止")

    # -- 急停：立即停止驱动（不依赖 UI 选择，随时可用）--
    def _estop(self):
        self.append("\n[急停] 立即停止电机驱动 ...")
        self.set_status("急停中")
        self.cancel.set()                      # 让可能在跑的前台命令循环尽快退出
        with self.lock:
            proc = self.drive_proc
            port, mid = self.drive_port, self.drive_id
        if proc is not None and proc.poll() is None:
            self.append("[急停] 终止驱动子进程 ...")
            try:
                proc.terminate()
                try:
                    proc.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    proc.kill()
            except Exception as e:
                self.append(f"[警告] 终止驱动失败: {e}")
        # 补一段 mode=0 停止脉冲，确保电机真正停下（用驱动时记录的 port/id）
        if port and mid is not None:
            cmd = [MOTOR_CTRL, port, str(mid), "stop", "500"]
            self.append(f"[急停] 停止脉冲 id={mid} @ {port}")
            try:
                p = self._spawn(cmd)
                for line in p.stdout:
                    self.append(line.rstrip("\n"))
                p.wait()
            except FileNotFoundError as e:
                self.append(f"[错误] {e}")
        with self.lock:
            self.drive_proc = None
        self.set_status("急停：驱动已停止")
        self.append("[急停] 完成。")

    # -- 取消 --
    def _cancel(self):
        self.cancel.set()
        with self.lock:
            proc = self.cmd_proc
        if proc is None or proc.poll() is not None:
            self.set_status("已请求取消，等待循环退出 ...")
            return
        self.append("[取消] 正在终止当前命令 ...")
        self.set_status("正在取消 ...")

        def killer(pr):
            try:
                pr.terminate()
                try:
                    pr.wait(timeout=2)
                    self.append("[取消] 已终止")
                except subprocess.TimeoutExpired:
                    pr.kill()
                    self.append("[取消] 已强制 kill")
            except Exception as e:
                self.append(f"[取消] 失败: {e}")

        threading.Thread(target=killer, args=(proc,), daemon=True).start()

    def _finish(self, title):
        with self.lock:
            self.busy = False
            self.cmd_proc = None
        self.set_status(f"完成: {title}")

    def _start_fg(self, fn, *args):
        with self.lock:
            self.busy = True
        self.cancel.clear()
        threading.Thread(target=fn, args=args, daemon=True).start()

    # -- 入口：派发动作 --
    def run(self, action, p):
        port = p.get("port", "")
        with self.lock:
            busy = self.busy
            driving = self.drive_proc is not None and self.drive_proc.poll() is None

        if action == "estop":   # 急停随时可用：不校验 busy/driving/port
            threading.Thread(target=self._estop, daemon=True).start()
            return True, "ok"

        fg = ("change_id", "switch_motor", "switch_motor_all",
              "read_angles", "scan_read")
        if action in fg or action in ("drive", "position"):
            if self.continuous:
                return False, "持续读取中：请先点「停止持续读取」"
            if busy:
                return False, "忙：请等待当前命令完成，或点「取消当前命令」"
            if driving:
                return False, "电机驱动中：请先点「停止」"

        # 这些动作不针对单个选中串口（全口扫描、姿态标定、持续读取等）
        no_port = ("cancel", "scan_read", "switch_motor_all", "cont_start",
                   "cont_stop", "capture_prone", "capture_stand", "save_config")
        if action not in no_port and not port:
            return False, "请先选择串口"

        if action == "scan_read":
            ports = list_ttyusb_ports()
            if not ports:
                return False, "未找到任何 /dev/ttyUSB*"
            if not os.path.isfile(MOTOR_CTRL):
                return False, f"未找到 {MOTOR_CTRL}"
            self._start_fg(self._scan_read, ports, "无损扫描全部串口 (读角度)")
        elif action == "switch_motor":
            # swmotor 把电机切回电机模式
            self._start_fg(self._foreground, [SWMOTOR, port], "切回电机模式", False, False, False)
        elif action == "switch_motor_all":
            ports = list_ttyusb_ports()
            if not ports:
                return False, "未找到任何 /dev/ttyUSB*"
            self._start_fg(self._multi, SWMOTOR, ports, "切回全部串口 (swmotor)", False, False)
        elif action == "change_id":
            old = str(p.get("old", "")).strip()
            new = str(p.get("new", "")).strip()
            if not (old.isdigit() and new.isdigit()):
                return False, "原 ID / 新 ID 必须是数字"
            if old == new:
                return False, "原 ID 与新 ID 不能相同"
            # changeid 同样让电机停在工厂(boot)模式
            self._start_fg(self._foreground, [CHANGEID, port, old, new],
                           f"修改电机 ID: {old} -> {new}", False, False, True)
        elif action == "read_angles":
            target = str(p.get("target", "all")).strip()
            if target != "all" and (not target.isdigit() or not MIN_MOTOR_ID <= int(target) <= MAX_MOTOR_ID):
                return False, f"电机 ID 必须在 {MIN_MOTOR_ID}~{MAX_MOTOR_ID}（或用 all）"
            if not os.path.isfile(MOTOR_CTRL):
                return False, f"未找到 {MOTOR_CTRL}"
            if self.factory_mode:
                self.append("[提示] 检测到之前做过「修改 ID」，电机可能仍在工厂(boot)模式；"
                            "工厂模式下不应答运控指令，读不到角度时请先点「切回电机模式 (swmotor)」。")
            title = "读取所有角度" if target == "all" else f"读取角度 id={target}"
            self._start_fg(self._foreground, [MOTOR_CTRL, port, target, "read"],
                           title, False, True)
        elif action == "drive":
            mid = str(p.get("id", "")).strip()
            if not mid.isdigit() or not MIN_MOTOR_ID <= int(mid) <= MAX_MOTOR_ID:
                return False, f"电机 ID 必须在 {MIN_MOTOR_ID}~{MAX_MOTOR_ID}"
            try:
                speed = float(p.get("speed", DEFAULT_SPEED))
            except (TypeError, ValueError):
                return False, "转速必须是数字（rad/s）"
            if not os.path.isfile(MOTOR_CTRL):
                return False, f"未找到 {MOTOR_CTRL}"
            threading.Thread(target=self._drive, args=(port, mid, speed),
                             daemon=True).start()
        elif action == "position":
            mid = str(p.get("id", "")).strip()
            if not mid.isdigit() or not MIN_MOTOR_ID <= int(mid) <= MAX_MOTOR_ID:
                return False, f"电机 ID 必须在 {MIN_MOTOR_ID}~{MAX_MOTOR_ID}"
            try:
                deg = float(p.get("deg", ""))
            except (TypeError, ValueError):
                return False, "转动角度必须是数字（°，正/负=正/反转）"
            if deg == 0:
                return False, "转动角度不能为 0"
            if abs(deg) > POS_MAX_DEG:
                return False, f"转动角度过大（|deg| 须 ≤ {POS_MAX_DEG:.0f}°），防止误填"
            try:
                vel = float(p.get("vel", POS_VEL_DEFAULT))
            except (TypeError, ValueError):
                return False, "速度必须是数字（关节 rad/s）"
            if not 0 < vel <= POS_VEL_MAX:
                return False, f"速度须在 0~{POS_VEL_MAX:.1f} 关节 rad/s 之间"
            try:
                kp = float(p.get("kp", POS_KP_DEFAULT))
            except (TypeError, ValueError):
                return False, "刚度 K_P 必须是数字"
            if not 0 <= kp <= POS_KP_MAX:
                return False, f"刚度 K_P 须在 0~{POS_KP_MAX:.1f} 之间"
            try:
                kw = float(p.get("kw", POS_KW_DEFAULT))
            except (TypeError, ValueError):
                return False, "阻尼 K_W 必须是数字"
            if not 0 <= kw <= POS_KW_MAX:
                return False, f"阻尼 K_W 须在 0~{POS_KW_MAX:.1f} 之间"
            try:
                t_ff = float(p.get("t", POS_T_DEFAULT))
            except (TypeError, ValueError):
                return False, "力矩前馈 T 必须是数字"
            if abs(t_ff) > POS_T_MAX:
                return False, f"力矩前馈 |T| 须 ≤ {POS_T_MAX:.1f} N·m（转子侧），防止力矩过大"
            on_arrive = str(p.get("arrive", "release")).strip()
            if on_arrive not in ("release", "hold", "keep"):
                return False, "到位行为只能是 release/hold/keep"
            if not os.path.isfile(MOTOR_CTRL):
                return False, f"未找到 {MOTOR_CTRL}"
            threading.Thread(
                target=self._position,
                args=(port, int(mid), deg, vel, kp, kw, t_ff, on_arrive),
                daemon=True).start()
        elif action == "stop":
            mid = str(p.get("id", "")).strip()
            if not mid.isdigit() or not MIN_MOTOR_ID <= int(mid) <= MAX_MOTOR_ID:
                return False, f"电机 ID 必须在 {MIN_MOTOR_ID}~{MAX_MOTOR_ID}"
            threading.Thread(target=self._stop, args=(port, mid), daemon=True).start()
        elif action == "cont_start":
            ports = list_ttyusb_ports()
            if not ports:
                return False, "未找到任何 /dev/ttyUSB*"
            if not os.path.isfile(MOTOR_CTRL):
                return False, f"未找到 {MOTOR_CTRL}"
            if busy or driving:
                return False, "忙/驱动中：请先停止再开启持续读取"
            self._cont_start()
        elif action == "cont_stop":
            self._cont_stop_now()
        elif action == "capture_prone":
            return self._capture_pose("prone")
        elif action == "capture_stand":
            return self._capture_pose("stand")
        elif action == "save_config":
            return self._save_config()
        elif action == "cancel":
            self._cancel()
        else:
            return False, f"未知操作: {action}"
        return True, "ok"


CTRL = Controller()


def check_tools():
    missing = [
        name for name, pth in (("changeid", CHANGEID), ("swmotor", SWMOTOR))
        if not os.path.isfile(pth)
    ]
    if missing:
        CTRL.append(f"[警告] 在 {TOOL_DIR} 下未找到: {', '.join(missing)}")
    if not os.path.isfile(MOTOR_CTRL):
        CTRL.append(f"[警告] 未找到 {MOTOR_CTRL}，请先在 build/ 执行: cmake .. && make motor_ctrl")
    CTRL.append("[提示] 只有「修改电机 ID」会让电机进入工厂模式（绿灯每秒快闪 3 次）；"
                "改完务必点「切回电机模式」，否则重新上电仍在工厂模式。其余操作（扫描/读角度/驱动）均不切模式。")
    CTRL.append("[就绪] 选择串口后即可操作。")


# ----------------------------------------------------------------- 前端页面
PAGE = """<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>GO-M8010-6 电机管理</title>
<style>
  :root { --bd:#d0d4da; --fg:#222; --muted:#888; --warn:#a60; --pri:#1769d6; }
  * { box-sizing: border-box; }
  body {
    margin: 0; padding: 16px;
    font-family: system-ui, -apple-system, "Noto Sans CJK SC",
                 "Microsoft YaHei", "WenQuanYi Micro Hei", sans-serif;
    color: var(--fg); background: #f5f6f8;
  }
  h1 { font-size: 18px; margin: 0 0 12px; }
  fieldset { border: 1px solid var(--bd); border-radius: 8px; margin: 0 0 12px; padding: 10px 12px; background: #fff; }
  legend { font-weight: 600; padding: 0 6px; color: #555; }
  details.panel {
    border: 1px solid var(--bd); border-radius: 8px; margin: 0 0 12px;
    padding: 6px 12px 10px; background: #fff;
  }
  details.panel > summary {
    font-weight: 600; color: #555; cursor: pointer; padding: 4px 0;
    list-style: revert;
  }
  details.panel[open] > summary { margin-bottom: 6px; }
  .row { display: flex; flex-wrap: wrap; align-items: center; gap: 8px 10px; }
  label { color: #555; }
  select, input { padding: 5px 7px; border: 1px solid var(--bd); border-radius: 6px; font-size: 14px; }
  input[type=number] { width: 90px; }
  button {
    padding: 6px 12px; border: 1px solid var(--bd); border-radius: 6px;
    background: #fff; cursor: pointer; font-size: 14px;
  }
  button:hover:not(:disabled) { border-color: var(--pri); color: var(--pri); }
  button:disabled { opacity: .5; cursor: not-allowed; }
  button.primary { background: var(--pri); color: #fff; border-color: var(--pri); }
  button.danger { background: #d64545; color: #fff; border-color: #d64545; }
  .tip { color: var(--warn); font-size: 13px; line-height: 1.6; }
  .muted { color: var(--muted); }
  table { border-collapse: collapse; width: 100%; font-size: 14px; }
  th, td { border: 1px solid var(--bd); padding: 5px 8px; text-align: center; }
  th { background: #eef1f5; }
  pre#log {
    height: 280px; overflow: auto; margin: 0; padding: 8px 10px;
    background: #1e1e1e; color: #e0e0e0; border-radius: 6px;
    font-family: "Noto Sans Mono CJK SC", "Sarasa Mono SC", ui-monospace, monospace;
    font-size: 12.5px; line-height: 1.45; white-space: pre-wrap; word-break: break-all;
  }
  #statusbar { display: flex; align-items: center; gap: 10px; margin-top: 10px; }
  #status { flex: 1; padding: 6px 10px; background: #fff; border: 1px solid var(--bd); border-radius: 6px; }
  .pill { font-size: 12px; padding: 2px 8px; border-radius: 10px; background: #eee; color: #555; }
  /* 右上角固定面板：驱动状态 + 大急停按钮（纵向堆叠，占右上空白区，不占整行）*/
  #driveBar{ position:fixed; top:12px; right:14px; z-index:1000; width:128px;
    display:flex; flex-direction:column; align-items:stretch; gap:8px;
    padding:10px; background:rgba(255,255,255,.96); border:1px solid var(--bd); border-radius:10px;
    box-shadow:0 2px 12px rgba(0,0,0,.18); transition:background .15s ease, border-color .15s ease; }
  #driveBar.on{ background:#fde8e8; border-color:#d64545; }
  #driveState{ text-align:center; font-size:14px; font-weight:700; padding:6px 8px; border-radius:9px;
    white-space:nowrap; background:#e3f4e7; color:#2a8a3e; }
  #driveBar.on #driveState{ background:#d64545; color:#fff; animation:dpulse 1s steps(1,end) infinite; }
  @keyframes dpulse{ 50%{ opacity:.45; } }
  #btnEstop{ width:100%; background:#d64545; color:#fff; border:2px solid #fff; border-radius:8px;
    font-size:18px; font-weight:800; letter-spacing:1px; line-height:1.25; padding:16px 8px; cursor:pointer;
    box-shadow:0 1px 4px rgba(0,0,0,.25); }
  #btnEstop:hover{ background:#bf3a3a; color:#fff; border-color:#fff; }
  /* 驱动中：整页红色边框警示（不挡点击）*/
  #driveEdge{ position:fixed; inset:0; border:6px solid #d64545; pointer-events:none; z-index:999; display:none; }
  #driveEdge.on{ display:block; animation:dpulse 1s steps(1,end) infinite; }
</style>
</head>
<body>
  <div id="driveBar">
    <span id="driveState">● 未驱动</span>
    <button id="btnEstop" type="button" onclick="estopNow()">⏹ 急停<br>STOP</button>
  </div>
  <div id="driveEdge"></div>
  <h1>GO-M8010-6 电机管理 <span class="muted" style="font-size:13px">（网页版）</span></h1>

  <div id="factoryWarn" style="display:none; margin:0 0 12px; padding:8px 12px;
       border:1px solid #e0a800; border-radius:8px; background:#fff8e1; color:#a60; font-size:13px;">
    ⚠️ 电机疑似处于工厂(boot)模式（刚做过修改 ID）。此模式下读不到角度、也无法驱动，
    请先点「2. 切回电机模式 (swmotor)」。
  </div>

  <fieldset>
    <legend>串口</legend>
    <div class="row">
      <label>串口号</label>
      <select id="port" style="min-width:160px"></select>
      <button class="action" onclick="refreshPorts()">刷新</button>
      <button class="action primary" onclick="api('scan_read')" title="遍历所有 ttyUSB 用 motor_ctrl 读角度，列出有响应的电机；保持电机模式，不切工厂模式">扫描电机 (读角度，不切模式)</button>
      <span class="pill">检测到 ID: <span id="detected">（无）</span></span>
    </div>
  </fieldset>

  <details class="panel">
    <summary>ID 管理（修改电机 ID，会进工厂模式）</summary>
    <div class="row">
      <label>原 ID</label><select id="oldId"></select>
      <label>新 ID</label><select id="newId"></select>
      <button class="action" onclick="doChangeId()">1. 修改电机 ID (changeid)</button>
      <span class="muted" style="font-size:13px">← 此操作会让电机进入工厂(boot)模式</span>
    </div>
    <div class="row" style="margin-top:8px">
      <button class="action primary" onclick="api('switch_motor')">2. 切回电机模式 (swmotor)</button>
      <button class="action" onclick="api('switch_motor_all')">切回全部 ttyUSB*</button>
    </div>
  </details>

  <fieldset>
    <legend>电机控制（需电机已处于电机模式）</legend>
    <div class="row">
      <label>电机 ID</label><select id="driveId"></select>
      <label title="电机【转子】转速 W（同 main.cpp）；输出端转速 = 此值 / 6.33。K_W 固定 0.05，转不动就调大此值">转子转速 W (rad/s)</label><input id="speed" type="number" step="0.1" value="6.28">
      <button class="action primary" onclick="doDrive()">▶ 驱动转动</button>
      <button id="btnStop" class="danger" onclick="doStop()" disabled>■ 停止</button>
      <span style="width:16px"></span>
      <button class="action" onclick="api('read_angles', {target:'all'})">📐 读取所有角度</button>
      <button class="action" onclick="api('read_angles', {target: document.getElementById('driveId').value})">读取选中 ID 角度</button>
    </div>
    <div class="row" style="margin-top:8px; border-top:1px dashed var(--bd); padding-top:8px">
      <label>位置控制：转动角度 (°)</label>
      <input id="posDeg" type="number" step="1" value="90" title="要转动的输出端关节角（相对当前位置）；正=正转，负=反转">
      <label title="关节角速度，决定多快走完轨迹；越小越慢越稳">速度 (rad/s)</label>
      <input id="posVel" type="number" step="0.1" value="0.5" style="width:80px">
      <label title="位置环刚度，越大越硬跟得越紧；抖动就调小（0~25.6）">刚度 K_P</label>
      <input id="posKp" type="number" step="0.5" value="2" style="width:64px">
      <label title="速度阻尼，抑制抖动；嗡鸣就调小（0~25.6）">阻尼 K_W</label>
      <input id="posKw" type="number" step="0.1" value="0.1" style="width:64px">
      <label title="力矩前馈（转子 N·m，可正负），常驻施加，可补偿重力/负载；0=不用">力矩 T</label>
      <input id="posT" type="number" step="0.05" value="0" style="width:64px">
      <label title="到位后行为：松力退出/持续保持/保留命令退出">到位后</label>
      <select id="posArrive" style="width:150px">
        <option value="release" selected>松力退出（默认）</option>
        <option value="hold">持续保持（点停止才退）</option>
        <option value="keep">保留命令退出（不松力）</option>
      </select>
      <button class="action primary" onclick="doPosition()" title="平滑位置控制（同 main.cpp）：读当前位置→按速度平滑转动到目标→按「到位后」处理（默认松力退出）">↻ 平滑转动 (相对)</button>
    </div>
    <div class="muted" style="font-size:12.5px; margin-top:6px">
      角度为<b>输出端关节角</b>（=转子角/6.33），相对当前位置，正=正转、负=反转。
      平滑位置控制（同 <code>main.cpp</code>）：读当前位置当起点 → 目标按速度平滑递增 →
      <code>K_P</code> 跟位置 + <code>K_W</code> 阻尼 + 速度前馈，到位后按下方「到位后」处理；点上方「■ 停止」随时中止并松力。
      <br>低速也丝滑。若<b>转动中抖</b>：调小 K_P；若<b>到位后嗡鸣</b>：调小 K_W；想更稳就减小速度。
      <b>力矩 T</b> 为转子侧前馈力矩（常驻、可正负），用于补偿重力/负载，0 即不用（幅值已限 ±3 N·m）。
      <br><b>到位后</b>：默认<b>松力退出</b>（电机松开，适合空载/测试）；要撑住负载选<b>持续保持</b>；
      <b>保留命令退出</b>则停发但不松力。注意：无论哪种，点「■ 停止」都会松力。
    </div>
  </fieldset>

  <fieldset>
    <legend>电机角度（read 读取，不驱动电机；显示输出端关节角，单位：度° = 转子角 / 6.33 × 180/π）</legend>
    <table>
      <thead><tr>
        <th>ID</th><th>关节角 (°)</th><th>温度 ℃</th><th>错误码</th>
      </tr></thead>
      <tbody id="angleBody">
        <tr><td colspan="6" class="muted">（暂无数据，点「读取所有角度」）</td></tr>
      </tbody>
    </table>
  </fieldset>

  <fieldset>
    <legend>姿态标定 → stand_config.json</legend>
    <div class="row">
      <button id="btnCont" class="calib" onclick="toggleCont()">▶ 持续读取</button>
      <span style="width:8px"></span>
      <button class="calib" onclick="api('capture_prone')">① 记录趴姿</button>
      <button class="calib" onclick="api('capture_stand')">② 记录站姿</button>
      <span style="width:8px"></span>
      <button class="calib" onclick="api('save_config')">💾 保存 stand_config.json</button>
      <button class="calib" onclick="loadSavedConfig()">📂 读取已保存配置</button>
      <span class="muted" style="font-size:13px">流程：开「持续读取」→ 摆趴姿点①→ 摆站姿点②→ 保存</span>
    </div>
    <div class="muted" style="font-size:12.5px; margin:6px 0 8px">
      实时方向 = 当前角相对【趴姿】的变化符号；标定方向 = sign(站姿 − 趴姿)，写入配置 dir 字段。
      <span style="color:var(--pri)">＋</span> 增大 / <span style="color:#d64545">－</span> 减小 / · 几乎不动。
    </div>
    <table>
      <thead><tr>
        <th>关节</th><th>ID</th><th>趴姿 (°)</th><th>站姿 (°)</th><th>当前 (°)</th>
        <th>实时方向<br>(趴→当前)</th><th>标定方向<br>(趴→站)</th>
      </tr></thead>
      <tbody id="calibBody">
        <tr><td colspan="7" class="muted">（加载 joint_map 中…）</td></tr>
      </tbody>
    </table>

    <div id="savedCfgWrap" style="margin-top:12px; display:none">
      <div style="font-weight:600; color:#555; margin-bottom:4px">已保存的 stand_config.json</div>
      <div id="savedCfgHead" class="muted" style="font-size:12.5px; margin-bottom:6px"></div>
      <table id="savedCfgTable">
        <thead><tr>
          <th>关节</th><th>ID</th><th>趴姿 (°)</th><th>站姿 (°)</th>
          <th>Δ关节 (°)</th><th>标定方向</th><th>已验证</th>
        </tr></thead>
        <tbody id="savedCfgBody"></tbody>
      </table>
      <details style="margin-top:6px">
        <summary class="muted" style="font-size:12.5px; cursor:pointer">原始 JSON</summary>
        <pre id="savedCfgRaw" style="max-height:240px; overflow:auto; margin:6px 0 0;
             padding:8px 10px; background:#1e1e1e; color:#e0e0e0; border-radius:6px;
             font-size:12px; line-height:1.45; white-space:pre-wrap; word-break:break-all"></pre>
      </details>
    </div>
  </fieldset>

  <p class="tip">
    提示：只有「修改电机 ID」会让电机进入工厂模式（背部绿灯每秒快闪 3 次）。
    操作前确保所有电机已停止、主机不再发送运动控制指令。
    改完必须点「切回电机模式」，否则重新上电仍在工厂模式。扫描/读角度/驱动都不会切模式。
  </p>

  <fieldset>
    <legend>输出</legend>
    <pre id="log"></pre>
  </fieldset>

  <div id="statusbar">
    <span id="status">就绪</span>
    <button id="btnCancel" class="action" onclick="api('cancel')" disabled>取消当前命令</button>
  </div>

<script>
  // 填充 ID 下拉框：驱动/读取用 MIN~MAX_MOTOR_ID（本机电机 1~12）；
  // 改 ID 仍允许 0~15，因为出厂未设 ID 的电机会以 15 出现（手册 §6.3）。
  const MIN_MOTOR_ID = __MIN_MOTOR_ID__;
  const MAX_MOTOR_ID = __MAX_MOTOR_ID__;
  function fillIds(sel, minId, maxId){
    for (let i=minId;i<=maxId;i++){ const o=document.createElement('option'); o.value=i; o.textContent=i; sel.appendChild(o); }
  }
  fillIds(document.getElementById('oldId'), 0, 15);
  fillIds(document.getElementById('newId'), 0, 15);
  fillIds(document.getElementById('driveId'), MIN_MOTOR_ID, MAX_MOTOR_ID);
  document.getElementById('oldId').value = '0';
  document.getElementById('newId').value = '1';

  let logIndex = 0;
  let lastDetected = '';
  let lastActivePort = '';
  let jointMap = null;
  let contOn = false;
  const DIR_DEAD = 0.01;  // rad，与后端一致：变化小于此值视为「未动」

  async function loadJointMap(){
    try{
      const r = await fetch('/api/jointmap'); jointMap = await r.json();
    }catch(e){ jointMap = {}; }
    updateCalib({});
  }
  function toggleCont(){ api(contOn ? 'cont_stop' : 'cont_start'); }

  // 读取并显示磁盘上已保存的 stand_config.json（只读展示，不影响当前标定状态）
  async function loadSavedConfig(){
    let j;
    try{
      const r = await fetch('/api/standconfig'); j = await r.json();
    }catch(e){ alert('读取失败: ' + e); return; }
    const wrap = document.getElementById('savedCfgWrap');
    const head = document.getElementById('savedCfgHead');
    const body = document.getElementById('savedCfgBody');
    const raw  = document.getElementById('savedCfgRaw');
    wrap.style.display = '';
    if (!j.exists){
      head.textContent = '尚未保存：' + (j.path || 'stand_config.json') + '（先点「💾 保存」生成）';
      body.innerHTML = ''; raw.textContent = ''; return;
    }
    if (j.error){
      head.textContent = '读取失败：' + j.error + '（' + (j.path||'') + '）';
      body.innerHTML = ''; raw.textContent = ''; return;
    }
    const cfg = j.config || {};
    head.textContent = '机器人=' + (cfg.robot||'—') + '  减速比=' + (cfg.gear_ratio||'—') +
                       '  生成于=' + (cfg._generated || j.mtime || '—') +
                       '  关节数=' + ((cfg.joints||[]).length) + '  文件=' + (j.path||'');
    const f2 = (v,n) => (v===null||v===undefined||v==='') ? '<span class="muted">—</span>' : Number(v).toFixed(n);
    const gear = cfg.gear_ratio || 6.33;
    const f2d = (rad,n) => (rad===null||rad===undefined||rad==='') ? '<span class="muted">—</span>' : Number(rad/gear*180/Math.PI).toFixed(n);  // 转子rad -> 关节度
    const dirCell = d => d===1 ? '<span style="color:var(--pri);font-weight:600">＋</span>'
                       : d===-1 ? '<span style="color:#d64545;font-weight:600">－</span>'
                       : d===0 ? '<span class="muted">·</span>' : '<span class="muted">—</span>';
    const verCell = v => v===true ? '✓' : v===false ? '<span style="color:#d64545">✗</span>' : '<span class="muted">—</span>';
    let html = '';
    for (const jt of (cfg.joints||[])){
      html += '<tr><td>'+(jt.name||'')+'</td><td>'+(jt.id??'')+'</td>'+
              '<td>'+f2(jt.prone_deg,2)+'</td><td>'+f2(jt.stand_deg,2)+'</td>'+
              '<td>'+f2d(jt.delta_rotor,2)+'</td><td>'+dirCell(jt.dir)+'</td>'+
              '<td>'+verCell(jt.verified)+'</td></tr>';
    }
    body.innerHTML = html || '<tr><td colspan="7" class="muted">（配置里没有 joints）</td></tr>';
    raw.textContent = JSON.stringify(cfg, null, 2);
  }

  async function refreshPorts(){
    try{
      const r = await fetch('/api/ports'); const j = await r.json();
      const sel = document.getElementById('port'); const cur = sel.value;
      sel.innerHTML='';
      (j.ports||[]).forEach(p=>{ const o=document.createElement('option'); o.value=p; o.textContent=p; sel.appendChild(o); });
      if (cur && (j.ports||[]).includes(cur)) sel.value = cur;
    }catch(e){}
  }

  async function api(action, params){
    params = params || {};
    params.action = action;
    params.port = document.getElementById('port').value;
    try{
      const r = await fetch('/api/run', {method:'POST', headers:{'Content-Type':'application/json'}, body: JSON.stringify(params)});
      const j = await r.json();
      if (!j.ok) alert(j.message || '操作失败');
    }catch(e){ alert('请求失败: ' + e); }
  }

  function doChangeId(){
    const oldv = document.getElementById('oldId').value;
    const newv = document.getElementById('newId').value;
    const port = document.getElementById('port').value;
    if (oldv === newv){ alert('原 ID 与新 ID 不能相同'); return; }
    if (!confirm('将串口 ' + port + ' 上所有 ID=' + oldv + ' 的电机改为 ID=' + newv +
                 '？\\n\\n请确认电机已停止、主机已不再发送控制指令。')) return;
    api('change_id', {old: oldv, new: newv});
  }
  function doDrive(){ api('drive', {id: document.getElementById('driveId').value, speed: document.getElementById('speed').value}); }
  function doStop(){ api('stop', {id: document.getElementById('driveId').value}); }
  function estopNow(){ api('estop'); }   // 急停：立即停止驱动，无需确认
  function doPosition(){
    api('position', {
      id:  document.getElementById('driveId').value,
      deg: document.getElementById('posDeg').value,
      vel: document.getElementById('posVel').value,
      kp:  document.getElementById('posKp').value,
      kw:  document.getElementById('posKw').value,
      t:   document.getElementById('posT').value,
      arrive: document.getElementById('posArrive').value,
    });
  }

  function updateButtons(s){
    // 持续读取期间也锁住普通操作（标定按钮 .calib 不受影响，便于记录/停止）
    const lock = s.busy || s.driving || s.continuous;
    document.querySelectorAll('button.action').forEach(b => { b.disabled = lock; });
    document.querySelectorAll('button.primary').forEach(b => { b.disabled = lock; });
    document.getElementById('btnStop').disabled = !s.driving;
    document.getElementById('btnCancel').disabled = !s.busy;
    contOn = !!s.continuous;
    const bc = document.getElementById('btnCont');
    bc.textContent = contOn ? '■ 停止持续读取' : '▶ 持续读取';
    bc.classList.toggle('danger', contOn);
    const driving = !!s.driving;
    document.getElementById('driveState').textContent = driving ? '● 驱动中' : '● 未驱动';
    document.getElementById('driveBar').classList.toggle('on', driving);
    document.getElementById('driveEdge').classList.toggle('on', driving);
  }

  function updateDetected(ids){
    document.getElementById('detected').textContent = (ids && ids.length) ? ids.join(', ') : '（无）';
    const key = JSON.stringify(ids || []);
    if (key === lastDetected) return;
    lastDetected = key;
    if (ids && ids.length){
      document.getElementById('oldId').value = String(ids[0]);
      document.getElementById('driveId').value = String(ids[0]);
    }
  }

  function updateActivePort(p){
    // 扫描到电机后，自动把端口下拉切到那根串口（仅在变化时执行，避免打断手动选择）
    if (!p || p === lastActivePort) return;
    lastActivePort = p;
    const sel = document.getElementById('port');
    let found = false;
    for (const o of sel.options) if (o.value === p) found = true;
    if (!found){ const o=document.createElement('option'); o.value=p; o.textContent=p; sel.appendChild(o); }
    sel.value = p;
  }

  function updateAngles(rows){
    const tb = document.getElementById('angleBody');
    if (!rows || !rows.length){
      tb.innerHTML = '<tr><td colspan="4" class="muted">（暂无数据，点「读取所有角度」）</td></tr>';
      return;
    }
    const f = (v,n) => { const x = parseFloat(v); return isNaN(x) ? '—' : x.toFixed(n); };
    tb.innerHTML = '';
    for (const r of rows){
      const tr = document.createElement('tr');
      tr.innerHTML = '<td>'+r.id+'</td><td>'+f(r.deg,2)+'</td>'+
                     '<td>'+(r.temp||'—')+'</td><td>'+(r.err||'—')+'</td>';
      tb.appendChild(tr);
    }
  }

  function degCell(rad){
    return (rad===null || rad===undefined) ? '<span class="muted">—</span>'
                                           : (rad*180/Math.PI).toFixed(2);
  }
  function dirSym(diffRad){
    if (diffRad===null || diffRad===undefined || isNaN(diffRad)) return '<span class="muted">—</span>';
    if (Math.abs(diffRad) < DIR_DEAD) return '<span class="muted">·</span>';
    return diffRad > 0 ? '<span style="color:var(--pri);font-weight:600">＋</span>'
                       : '<span style="color:#d64545;font-weight:600">－</span>';
  }
  function updateCalib(s){
    const tb = document.getElementById('calibBody');
    if (!jointMap || !jointMap.joints || !jointMap.joints.length){
      tb.innerHTML = '<tr><td colspan="7" class="muted">（未找到 joint_map.default.json，无法显示关节表）</td></tr>';
      return;
    }
    const cur = {};
    (s.angles||[]).forEach(r => { if (r.ok!==false) cur[String(r.id)] = parseFloat(r.joint); });
    const prone = s.pose_prone || {};
    const stand = s.pose_stand || {};
    let html = '';
    for (const j of jointMap.joints){
      const id = String(j.id);
      const p  = (id in prone) ? prone[id] : null;
      const st = (id in stand) ? stand[id] : null;
      const c  = (id in cur)   ? cur[id]   : null;
      const liveDir = (p!==null && c!==null) ? dirSym(c - p) : '<span class="muted">—</span>';
      const calDir  = (p!==null && st!==null) ? dirSym(st - p) : '<span class="muted">—</span>';
      html += '<tr><td>'+(j.name||'')+'</td><td>'+j.id+'</td>'+
              '<td>'+degCell(p)+'</td><td>'+degCell(st)+'</td><td>'+degCell(c)+'</td>'+
              '<td>'+liveDir+'</td><td>'+calDir+'</td></tr>';
    }
    tb.innerHTML = html;
  }

  async function poll(){
    try{
      const r = await fetch('/api/state?since=' + logIndex);
      const s = await r.json();
      const log = document.getElementById('log');
      if (s.log && s.log.length){
        const atBottom = log.scrollTop + log.clientHeight >= log.scrollHeight - 24;
        log.textContent += s.log.join('\\n') + '\\n';
        if (atBottom) log.scrollTop = log.scrollHeight;
      }
      logIndex = s.log_index;
      document.getElementById('status').textContent = s.status;
      updateButtons(s);
      updateActivePort(s.active_port);
      updateDetected(s.detected_ids);
      updateAngles(s.angles);
      updateCalib(s);
      document.getElementById('factoryWarn').style.display = s.factory_mode ? '' : 'none';
    }catch(e){}
  }

  refreshPorts();
  loadJointMap();
  poll();
  setInterval(poll, 400);
</script>
</body>
</html>
"""


# ----------------------------------------------------------------- HTTP 处理
class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass  # 静默，避免污染控制台

    def _send(self, data, content_type):
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _json(self, obj, code=200):
        data = json.dumps(obj, ensure_ascii=False).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        u = urlparse(self.path)
        if u.path in ("/", "/index.html"):
            html = (PAGE.replace("__MIN_MOTOR_ID__", str(MIN_MOTOR_ID))
                        .replace("__MAX_MOTOR_ID__", str(MAX_MOTOR_ID)))
            self._send(html.encode("utf-8"), "text/html; charset=utf-8")
        elif u.path == "/api/ports":
            self._json({"ports": list_serial_ports()})
        elif u.path == "/api/jointmap":
            self._json(CTRL.joint_map or {})
        elif u.path == "/api/standconfig":
            self._json(CTRL.read_saved_config())
        elif u.path == "/api/state":
            q = parse_qs(u.query)
            try:
                since = int(q.get("since", ["0"])[0])
            except (ValueError, TypeError):
                since = 0
            self._json(CTRL.snapshot(since))
        else:
            self._json({"error": "not found"}, 404)

    def do_POST(self):
        u = urlparse(self.path)
        if u.path == "/api/run":
            length = int(self.headers.get("Content-Length", "0") or 0)
            body = self.rfile.read(length) if length else b"{}"
            try:
                p = json.loads(body.decode("utf-8") or "{}")
            except Exception:
                p = {}
            ok, msg = CTRL.run(p.get("action", ""), p)
            self._json({"ok": ok, "message": msg})
        else:
            self._json({"error": "not found"}, 404)


def main():
    host = os.environ.get("HOST", "127.0.0.1")
    port = int(os.environ.get("PORT", "8000"))
    check_tools()
    srv = ThreadingHTTPServer((host, port), Handler)
    print(f"电机管理网页已启动:  http://{host}:{port}")
    if host not in ("127.0.0.1", "localhost"):
        print("⚠️  正在监听非本机地址，局域网内任何人都能控制电机，请注意安全。")
    print("按 Ctrl+C 退出。")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\n已退出。")
    finally:
        srv.server_close()


if __name__ == "__main__":
    main()
