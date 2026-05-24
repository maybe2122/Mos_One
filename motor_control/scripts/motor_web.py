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
import os
import re
import subprocess
import threading
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
DEFAULT_SPEED = 6.28 * 6.33
NEED_SUDO = os.geteuid() != 0
MAX_LOG = 4000  # 日志缓冲最多保留的行数


# ----------------------------------------------------------------- 工具函数
def list_serial_ports():
    ports = []
    for name in os.listdir("/dev"):
        if name.startswith(("ttyUSB", "ttyACM", "ttyS")):
            ports.append("/dev/" + name)
    ports.sort()
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
        self.drive_proc = None    # 持续运行的驱动子进程
        self.cancel = threading.Event()
        self.detected_ids = []
        self.angles = []
        self.status = "就绪"

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
    def _foreground(self, cmd, title, post_parse_ids=False, parse_angles=False):
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
            else:
                self.append("==> 无电机响应（确认已处于电机模式且接线正常）")

        self._finish(title)

    # -- 多串口批量（扫描全部 / 切回全部） --
    def _multi(self, tool, ports, title, post_parse_ids):
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
        self._finish(title)

    # -- 驱动 / 停止 --
    def _drive(self, port, mid, speed):
        cmd = [MOTOR_CTRL, port, str(mid), "drive", f"{speed}", "0"]
        shown = (["sudo"] + cmd) if NEED_SUDO else cmd
        self.append("\n" + "=" * 60)
        self.append(f"[驱动 id={mid} speed={speed}]  $ {' '.join(shown)}")
        self.set_status(f"驱动中: id={mid} speed={speed}")
        try:
            proc = self._spawn(cmd)
        except FileNotFoundError as e:
            self.append(f"[错误] {e}")
            self.set_status("驱动已停止")
            return
        with self.lock:
            self.drive_proc = proc
        threading.Thread(target=self._drive_reader, args=(proc,), daemon=True).start()

    def _drive_reader(self, proc):
        for line in proc.stdout:
            self.append(line.rstrip("\n"))
        proc.wait()
        self.append(f"[驱动进程退出码 {proc.returncode}]")
        with self.lock:
            self.drive_proc = None
        self.set_status("驱动已停止")

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

        fg = ("view_id", "view_id_all", "change_id", "switch_motor",
              "switch_motor_all", "read_angles")
        if action in fg or action == "drive":
            if busy:
                return False, "忙：请等待当前命令完成，或点「取消当前命令」"
            if driving:
                return False, "电机驱动中：请先点「停止」"

        if action != "cancel" and not port:
            return False, "请先选择串口"

        if action == "view_id":
            self._start_fg(self._foreground, [SWBOOT, port], "查看电机 ID", True, False)
        elif action == "view_id_all":
            ports = list_ttyusb_ports()
            if not ports:
                return False, "未找到任何 /dev/ttyUSB*"
            self._start_fg(self._multi, SWBOOT, ports, "扫描全部串口 (swboot)", True)
        elif action == "switch_motor":
            self._start_fg(self._foreground, [SWMOTOR, port], "切回电机模式", False, False)
        elif action == "switch_motor_all":
            ports = list_ttyusb_ports()
            if not ports:
                return False, "未找到任何 /dev/ttyUSB*"
            self._start_fg(self._multi, SWMOTOR, ports, "切回全部串口 (swmotor)", False)
        elif action == "change_id":
            old = str(p.get("old", "")).strip()
            new = str(p.get("new", "")).strip()
            if not (old.isdigit() and new.isdigit()):
                return False, "原 ID / 新 ID 必须是数字"
            if old == new:
                return False, "原 ID 与新 ID 不能相同"
            self._start_fg(self._foreground, [CHANGEID, port, old, new],
                           f"修改电机 ID: {old} -> {new}", False, False)
        elif action == "read_angles":
            target = str(p.get("target", "all")).strip()
            if target != "all" and (not target.isdigit() or not 0 <= int(target) <= 15):
                return False, "电机 ID 必须在 0~15（或用 all）"
            if not os.path.isfile(MOTOR_CTRL):
                return False, f"未找到 {MOTOR_CTRL}"
            title = "读取所有角度" if target == "all" else f"读取角度 id={target}"
            self._start_fg(self._foreground, [MOTOR_CTRL, port, target, "read"],
                           title, False, True)
        elif action == "drive":
            mid = str(p.get("id", "")).strip()
            if not mid.isdigit() or not 0 <= int(mid) <= 15:
                return False, "电机 ID 必须在 0~15"
            try:
                speed = float(p.get("speed", DEFAULT_SPEED))
            except (TypeError, ValueError):
                return False, "转速必须是数字（rad/s）"
            if not os.path.isfile(MOTOR_CTRL):
                return False, f"未找到 {MOTOR_CTRL}"
            threading.Thread(target=self._drive, args=(port, mid, speed),
                             daemon=True).start()
        elif action == "stop":
            mid = str(p.get("id", "")).strip()
            if not mid.isdigit() or not 0 <= int(mid) <= 15:
                return False, "电机 ID 必须在 0~15"
            threading.Thread(target=self._stop, args=(port, mid), daemon=True).start()
        elif action == "cancel":
            self._cancel()
        else:
            return False, f"未知操作: {action}"
        return True, "ok"


CTRL = Controller()


def check_tools():
    missing = [
        name for name, pth in (("swboot", SWBOOT), ("changeid", CHANGEID),
                               ("swmotor", SWMOTOR))
        if not os.path.isfile(pth)
    ]
    if missing:
        CTRL.append(f"[警告] 在 {TOOL_DIR} 下未找到: {', '.join(missing)}")
    if not os.path.isfile(MOTOR_CTRL):
        CTRL.append(f"[警告] 未找到 {MOTOR_CTRL}，请先在 build/ 执行: cmake .. && make motor_ctrl")
    CTRL.append("[提示] 查看 / 修改 ID 会让电机进入工厂模式（绿灯每秒快闪 3 次）；"
                "完成后务必点「切回电机模式」，否则重新上电仍在工厂模式。")
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
</style>
</head>
<body>
  <h1>GO-M8010-6 电机管理 <span class="muted" style="font-size:13px">（网页版）</span></h1>

  <fieldset>
    <legend>串口</legend>
    <div class="row">
      <label>串口号</label>
      <select id="port" style="min-width:160px"></select>
      <button class="action" onclick="refreshPorts()">刷新</button>
      <button class="action" onclick="api('view_id_all')">扫描全部 ttyUSB*</button>
      <span class="pill">检测到 ID: <span id="detected">（无）</span></span>
    </div>
  </fieldset>

  <fieldset>
    <legend>ID 管理</legend>
    <div class="row">
      <button class="action" onclick="api('view_id')">1. 查看电机 ID (swboot)</button>
      <span style="width:16px"></span>
      <label>原 ID</label><select id="oldId"></select>
      <label>新 ID</label><select id="newId"></select>
      <button class="action" onclick="doChangeId()">2. 修改电机 ID (changeid)</button>
    </div>
    <div class="row" style="margin-top:8px">
      <button class="action" onclick="api('switch_motor')">3. 切回电机模式 (swmotor)</button>
      <button class="action" onclick="api('switch_motor_all')">切回全部 ttyUSB*</button>
    </div>
  </fieldset>

  <fieldset>
    <legend>电机控制（需电机已处于电机模式）</legend>
    <div class="row">
      <label>电机 ID</label><select id="driveId"></select>
      <label>转速 (rad/s)</label><input id="speed" type="number" step="0.001" value="39.752">
      <button class="action primary" onclick="doDrive()">▶ 驱动转动</button>
      <button id="btnStop" class="danger" onclick="doStop()" disabled>■ 停止</button>
      <span style="width:16px"></span>
      <button class="action" onclick="api('read_angles', {target:'all'})">📐 读取所有角度</button>
      <button class="action" onclick="api('read_angles', {target: document.getElementById('driveId').value})">读取选中 ID 角度</button>
    </div>
  </fieldset>

  <fieldset>
    <legend>电机角度（read 读取，不驱动电机；关节角 = 转子角 / 6.33）</legend>
    <table>
      <thead><tr>
        <th>ID</th><th>关节 (rad)</th><th>关节 (°)</th><th>转子 (rad)</th><th>温度 ℃</th><th>错误码</th>
      </tr></thead>
      <tbody id="angleBody">
        <tr><td colspan="6" class="muted">（暂无数据，点「读取所有角度」）</td></tr>
      </tbody>
    </table>
  </fieldset>

  <p class="tip">
    提示：查看 / 修改 ID 会让电机进入工厂模式（背部绿灯每秒快闪 3 次）。
    操作前确保所有电机已停止、主机不再发送运动控制指令。
    完成后必须点「切回电机模式」，否则重新上电仍在工厂模式。
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
  // 0~15 填充三个 ID 下拉框
  for (const sid of ['oldId','newId','driveId']) {
    const sel = document.getElementById(sid);
    for (let i=0;i<16;i++){ const o=document.createElement('option'); o.value=i; o.textContent=i; sel.appendChild(o); }
  }
  document.getElementById('oldId').value = '0';
  document.getElementById('newId').value = '1';

  let logIndex = 0;
  let lastDetected = '';

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

  function updateButtons(s){
    const lock = s.busy || s.driving;
    document.querySelectorAll('button.action').forEach(b => { b.disabled = lock; });
    document.querySelectorAll('button.primary').forEach(b => { b.disabled = lock; });
    document.getElementById('btnStop').disabled = !s.driving;
    document.getElementById('btnCancel').disabled = !s.busy;
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

  function updateAngles(rows){
    const tb = document.getElementById('angleBody');
    if (!rows || !rows.length){
      tb.innerHTML = '<tr><td colspan="6" class="muted">（暂无数据，点「读取所有角度」）</td></tr>';
      return;
    }
    const f = (v,n) => { const x = parseFloat(v); return isNaN(x) ? '—' : x.toFixed(n); };
    tb.innerHTML = '';
    for (const r of rows){
      const tr = document.createElement('tr');
      tr.innerHTML = '<td>'+r.id+'</td><td>'+f(r.joint,4)+'</td><td>'+f(r.deg,2)+
                     '</td><td>'+f(r.rotor,4)+'</td><td>'+(r.temp||'—')+'</td><td>'+(r.err||'—')+'</td>';
      tb.appendChild(tr);
    }
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
      updateDetected(s.detected_ids);
      updateAngles(s.angles);
    }catch(e){}
  }

  refreshPorts();
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
            self._send(PAGE.encode("utf-8"), "text/html; charset=utf-8")
        elif u.path == "/api/ports":
            self._json({"ports": list_serial_ports()})
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
