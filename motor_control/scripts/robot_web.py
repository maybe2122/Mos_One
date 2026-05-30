#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""四足机器人操控网页 —— 站立（趴 → 站）。

仿手柄的操控面板，本期只实现「站立」一个动作的完整闭环：
  1. 趴姿标定：机器人自然趴下，读取全部 12 关节转子角 -> prone
  2. 站姿标定：人手扶撑成站立姿态，读取全部 12 关节转子角 -> stand
  3. 计算 趴->站 每关节转动量 delta=stand-prone（带符号=方向），存配置文件
  4. 站立：先读当前角，逐关节与配置趴姿比 |Δ|<阈值 才放行；放行后目标=当前+delta，
     按关节限速（默认 0.1 rad/s）做平滑插值，通过 motor_ctrl servo 流式发位置指令。

底层复用 motor_control/Linux/build/motor_ctrl：
  - 读角度：motor_ctrl <port> <id> read
  - 流式位置伺服：motor_ctrl <port> servo <kp> <kw>（stdin 喂目标位置，插值在本脚本里算）

纯标准库（http.server），默认仅监听 127.0.0.1，避免把机器人控制暴露到局域网。
运行：python3 scripts/robot_web.py  然后浏览器开 http://127.0.0.1:8000
设计文档见 motor_control/doc/四足站立控制设计.md。
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

# --- 路径与常量（与 motor_web.py 对齐） ---
_HERE = os.path.dirname(os.path.abspath(__file__))
_DEFAULT_SDK_ROOT = os.path.normpath(os.path.join(_HERE, "..", "Linux"))
SDK_ROOT = os.environ.get("UNITREE_MOTOR_SDK", _DEFAULT_SDK_ROOT)
MOTOR_CTRL = os.path.join(SDK_ROOT, "build", "motor_ctrl")

CONFIG_DIR = os.path.normpath(os.path.join(_HERE, "..", "config"))
JOINT_MAP_DEFAULT = os.path.join(CONFIG_DIR, "joint_map.default.json")
STAND_CONFIG = os.path.join(CONFIG_DIR, "stand_config.json")

GEAR_RATIO = 6.33
SUDO_PASSWORD = "1"
NEED_SUDO = os.geteuid() != 0
MAX_LOG = 4000


# ----------------------------------------------------------------- 工具函数
def parse_angle_line(line):
    """解析 motor_ctrl read 的输出，取转子角(rotor=data.Pos)。
        ANGLE id=0 ok=1 rotor=1.234 joint=0.195 deg=11.17 temp=30 err=0
    """
    m = re.match(r"\s*ANGLE\s+id=(\d+)\s+ok=([01])(.*)", line)
    if not m:
        return None
    mid = int(m.group(1))
    ok = m.group(2) == "1"
    rest = m.group(3)

    def grab(key):
        mm = re.search(rf"{key}=(-?\d+(?:\.\d+)?)", rest)
        return float(mm.group(1)) if mm else None

    if ok:
        return {"id": mid, "ok": True, "rotor": grab("rotor"),
                "temp": grab("temp"), "err": grab("err")}
    return {"id": mid, "ok": False}


def load_joint_template():
    """读默认映射模板，返回 (joints, execute)。"""
    with open(JOINT_MAP_DEFAULT, "r", encoding="utf-8") as f:
        j = json.load(f)
    return j["joints"], j.get("execute", {})


def joints_by_port(joints):
    """按串口分组，保持顺序。返回 {port: [joint, ...]}。"""
    groups = {}
    for jt in joints:
        groups.setdefault(jt["port"], []).append(jt)
    return groups


def parse_opt_gain(raw):
    """解析前端传来的可选增益（K_P / K_W）：
       空 -> None（表示沿用配置 execute 里的默认值）；
       非数字 -> "ERR"（调用方据此报错）；
       合法 -> clamp 到 [0, 25.599]（手册 K_P/K_W 上限）。
    """
    s = str(raw).strip()
    if s == "":
        return None
    try:
        v = float(s)
    except (TypeError, ValueError):
        return "ERR"
    return max(0.0, min(25.599, v))


# ----------------------------------------------------------------- 控制器
class RobotController:
    def __init__(self):
        self.lock = threading.RLock()
        self.log = []
        self.dropped = 0
        self.busy = False           # 标定 / 读取等短任务进行中
        self.standing_thread = None  # 站立执行线程
        self.abort = threading.Event()
        self.servo_procs = {}        # port -> Popen（流式伺服进程）
        self.hold_scope = ""         # 当前 servo 锁位保持的范围描述（空=已松开）；供前端「是否松开」显示
        self.notice = None           # 供前端弹窗的后台通知 {id,text,level}；轮询到新 id 弹一次
        self.notice_id = 0

        # 标定与配置数据（关节角均以「转子角 rad」存储，与 motor_ctrl 的 Pos 一致）
        self.joints, self.execute = load_joint_template()
        self.prone = {}              # name -> rotor
        self.stand = {}              # name -> rotor
        self.current = {}            # name -> rotor（最近一次读取）
        self.current_ok = {}         # name -> bool
        self.last_dev = {}           # name -> 与配置趴姿的偏差（校验用）
        self.feedback = {}           # name -> {vel,tau,errd,temp,...} servo 实时反馈（保持/运动时）
        self._id2name = {int(jt["id"]): jt["name"] for jt in self.joints}  # 电机 id -> 关节名
        self._config_mtime = None    # stand_config.json 的 mtime，用于检测外部更新后自动重载
        self.config = self._load_config()
        self.phase = "IDLE"          # IDLE/PRONE_DONE/STAND_DONE/CONFIGURED/EXECUTING/STANDING/ERROR
        self.status = "就绪"
        if self.config:
            self.phase = "CONFIGURED"

    # -- 日志 --
    def append(self, line):
        with self.lock:
            self.log.append(str(line))
            over = len(self.log) - MAX_LOG
            if over > 0:
                del self.log[:over]
                self.dropped += over

    def set_status(self, s):
        with self.lock:
            self.status = s

    def _notify(self, text, level="bad"):
        """记录一条供前端弹窗的通知。后台线程里的拒绝/异常无法通过 api() 返回值
        告知页面（动作早已异步返回 ok），改为写入 notice，前端轮询到新的 id 就 alert 一次。"""
        with self.lock:
            self.notice_id += 1
            self.notice = {"id": self.notice_id, "text": str(text), "level": level}

    def _load_config(self):
        if os.path.isfile(STAND_CONFIG):
            try:
                with open(STAND_CONFIG, "r", encoding="utf-8") as f:
                    cfg = json.load(f)
                try:
                    self._config_mtime = os.path.getmtime(STAND_CONFIG)
                except OSError:
                    self._config_mtime = None
                return cfg
            except Exception as e:
                self.append(f"[警告] 读取 {STAND_CONFIG} 失败: {e}")
        self._config_mtime = None
        return None

    def _reload_config_now(self):
        """手动从磁盘重新加载 stand_config.json，并打印一行摘要（供页面「重新加载配置」按钮）。"""
        cfg = self._load_config()   # 内部会更新 self._config_mtime
        with self.lock:
            self.config = cfg
            if cfg is not None and self.phase == "IDLE":
                self.phase = "CONFIGURED"
        if cfg is None:
            self.append(f"[配置] 未找到或无法读取 {STAND_CONFIG}（请先在 motor_web 标定并保存）。")
            return False, f"未找到/无法读取 {STAND_CONFIG}"
        js = cfg.get("joints", [])
        n_dir = sum(1 for j in js if j.get("dir") is not None)
        n_ver = sum(1 for j in js if j.get("verified"))
        self.append(f"[配置] 已重新加载 {STAND_CONFIG}：机器人={cfg.get('robot')} "
                    f"减速比={cfg.get('gear_ratio')} 关节={len(js)} 含方向={n_dir} "
                    f"已验证={n_ver} 生成于={cfg.get('_generated', '?')}")
        return True, "ok"

    def _refresh_config_if_changed(self):
        """若磁盘上的 stand_config.json 比内存里的新（被 motor_web 标定 / 手动改过），
        自动重载，保证关节角表里的趴姿/站姿始终跟配置文件一致。"""
        try:
            mtime = os.path.getmtime(STAND_CONFIG) if os.path.isfile(STAND_CONFIG) else None
        except OSError:
            mtime = None
        if mtime == self._config_mtime:
            return
        cfg = self._load_config()   # 内部会更新 self._config_mtime
        self.config = cfg
        if cfg is not None:
            if self.phase in ("IDLE",):
                self.phase = "CONFIGURED"
            self.append(f"[配置] 检测到 {STAND_CONFIG} 已更新，关节角表已按最新配置刷新。")

    # -- 状态快照 --
    def snapshot(self, since):
        with self.lock:
            self._refresh_config_if_changed()
            total = self.dropped + len(self.log)
            start = max(since, self.dropped)
            lines = self.log[start - self.dropped:]
            cfg_joints = {}
            if self.config:
                for j in self.config.get("joints", []):
                    cfg_joints[j["name"]] = j
            rows = []
            for jt in self.joints:
                name = jt["name"]
                cj = cfg_joints.get(name, {})
                fb = self.feedback.get(name, {})
                rows.append({
                    "name": name, "port": jt["port"], "id": jt["id"],
                    "prone": cj.get("prone_rotor", self.prone.get(name)),
                    "stand": cj.get("stand_rotor", self.stand.get(name)),
                    "delta": cj.get("delta_rotor"),
                    "current": self.current.get(name),
                    "ok": self.current_ok.get(name),
                    "dev": self.last_dev.get(name),
                    "verified": cj.get("verified", False),
                    "fb_vel": fb.get("vel"),     # 实测转子角速度 rad/s
                    "fb_tau": fb.get("tau"),     # 实测力矩 N·m
                    "fb_errd": fb.get("errd"),   # 跟踪误差（关节°）
                    "fb_temp": fb.get("temp"),   # 温度 ℃
                })
            holding = False
            for _p in self.servo_procs.values():
                try:
                    if _p.poll() is None:   # 进程还活着 = 电机仍带电锁位保持（未松开）
                        holding = True
                        break
                except Exception:
                    pass
            return {
                "phase": self.phase,
                "status": self.status,
                "busy": self.busy,
                "standing": self.standing_thread is not None
                            and self.standing_thread.is_alive(),
                "configured": self.config is not None,
                "holding": holding,
                "hold_scope": self.hold_scope,
                "notice": self.notice,
                "has_prone": bool(self.prone),
                "has_stand": bool(self.stand),
                "execute": (self.config or {}).get("execute", self.execute),
                "gear_ratio": (self.config or {}).get("gear_ratio", GEAR_RATIO),
                "joints": rows,
                "log": lines,
                "log_index": total,
            }

    # -- 子进程：一次性捕获（读角度） --
    def _run_capture(self, cmd, timeout=8):
        full = (["sudo", "-S", "-p", ""] + cmd) if NEED_SUDO else cmd
        try:
            r = subprocess.run(
                full, input=(SUDO_PASSWORD + "\n") if NEED_SUDO else None,
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, timeout=timeout,
            )
            return r.stdout or ""
        except FileNotFoundError as e:
            self.append(f"[错误] {e}")
            return ""
        except subprocess.TimeoutExpired:
            self.append(f"[超时] {' '.join(cmd)}")
            return ""

    # -- 读取全部关节转子角（按串口并行） --
    def _read_all(self, joints=None):
        if not os.path.isfile(MOTOR_CTRL):
            self.append(f"[错误] 未找到 {MOTOR_CTRL}，请先编译 motor_ctrl")
            return {}, {}
        results = {}
        oks = {}
        groups = joints_by_port(joints if joints is not None else self.joints)
        threads = []

        def worker(port, jts):
            for jt in jts:
                if self.abort.is_set():
                    return
                out = self._run_capture([MOTOR_CTRL, port, str(jt["id"]), "read"])
                rotor, ok = None, False
                for line in out.splitlines():
                    r = parse_angle_line(line)
                    if r and r["id"] == jt["id"]:
                        ok = r["ok"]
                        if ok:
                            rotor = r["rotor"]
                        break
                with self.lock:
                    results[jt["name"]] = rotor
                    oks[jt["name"]] = ok
                tag = f"rotor={rotor:.4f}" if ok and rotor is not None else "无响应"
                self.append(f"  [{jt['name']}] {port} id={jt['id']}: {tag}")

        for port, jts in groups.items():
            t = threading.Thread(target=worker, args=(port, jts), daemon=True)
            t.start()
            threads.append(t)
        for t in threads:
            t.join()
        return results, oks

    # -- 标定：读趴姿 / 读站姿 --
    def _calib(self, which):
        self.append("\n" + "=" * 56)
        label = "趴姿" if which == "prone" else "站姿"
        self.append(f"[标定-{label}] 读取全部 12 关节当前转子角 ...")
        self.set_status(f"标定中：读取{label}角度")
        res, oks = self._read_all()
        missing = [n for n, ok in oks.items() if not ok]
        with self.lock:
            self.current = dict(res)
            self.current_ok = dict(oks)
            if which == "prone":
                self.prone = {n: v for n, v in res.items() if oks.get(n)}
            else:
                self.stand = {n: v for n, v in res.items() if oks.get(n)}
            if missing:
                self.append(f"[警告] 以下关节无响应，未记入{label}: {', '.join(missing)}")
            else:
                self.append(f"[完成] {label}标定记录了全部 12 关节。")
            self._refresh_phase()
        self._finish(f"标定{label}")

    def _refresh_phase(self):
        if self.standing_thread is not None and self.standing_thread.is_alive():
            return
        if self.config is not None:
            self.phase = "CONFIGURED"
        elif self.prone and self.stand:
            self.phase = "STAND_DONE"
        elif self.prone:
            self.phase = "PRONE_DONE"
        else:
            self.phase = "IDLE"

    # -- 标定：计算 delta 并保存配置 --
    def _save_config(self):
        with self.lock:
            prone, stand = dict(self.prone), dict(self.stand)
        names = [jt["name"] for jt in self.joints]
        miss = [n for n in names if n not in prone or n not in stand]
        if miss:
            return False, f"以下关节缺少趴姿或站姿标定，无法保存: {', '.join(miss)}"

        joints_out = []
        for jt in self.joints:
            n = jt["name"]
            p, s = prone[n], stand[n]
            joints_out.append({
                "name": n, "port": jt["port"], "id": jt["id"],
                "prone_rotor": round(p, 6),
                "stand_rotor": round(s, 6),
                "delta_rotor": round(s - p, 6),
            })
        cfg = {
            "robot": "mos2026_2",
            "gear_ratio": GEAR_RATIO,
            "calibrated_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
            "encoder_note": "单圈绝对值编码器，绝对位置断电不变，配置跨上电有效；机械/零点变动后才需重标",
            "joints": joints_out,
            "execute": self.execute,
        }
        os.makedirs(CONFIG_DIR, exist_ok=True)
        with open(STAND_CONFIG, "w", encoding="utf-8") as f:
            json.dump(cfg, f, ensure_ascii=False, indent=2)
        with self.lock:
            self.config = cfg
            try:
                self._config_mtime = os.path.getmtime(STAND_CONFIG)
            except OSError:
                self._config_mtime = None
            self.phase = "CONFIGURED"
        self.append(f"\n[保存] 配置已写入 {STAND_CONFIG}")
        for j in joints_out:
            arrow = "↑" if j["delta_rotor"] >= 0 else "↓"
            pj = math.degrees(j["prone_rotor"] / GEAR_RATIO)
            sj = math.degrees(j["stand_rotor"] / GEAR_RATIO)
            dj = math.degrees(j["delta_rotor"] / GEAR_RATIO)
            self.append(f"  {j['name']}: 趴={pj:.2f}° 站={sj:.2f}° Δ={dj:+.2f}° {arrow}")
        return True, "配置已保存"

    # -- 单关节方向：confirm 确认 / unconfirm 取消确认 / invert 取反(翻转 delta/dir 并存盘) --
    def _set_dir(self, name, mode):
        cfg = self._load_config()
        if not cfg:
            return False, "没有配置文件，请先标定保存"
        target = next((j for j in cfg["joints"] if j.get("name") == name), None)
        if target is None:
            return False, f"配置里没有关节 {name}"

        if mode == "invert":
            dr = target.get("delta_rotor")
            if dr is None:
                return False, f"{name} 缺 delta_rotor，无法取反"
            target["delta_rotor"] = round(-dr, 6)
            pr = target.get("prone_rotor")
            if pr is not None:
                target["stand_rotor"] = round(pr + target["delta_rotor"], 6)
            if target.get("dir") in (1, -1):
                target["dir"] = -target["dir"]
            # 同步 motor_web 的关节角字段（站姿关于趴姿镜像）
            pra, sta = target.get("prone_rad"), target.get("stand_rad")
            if pra is not None and sta is not None:
                target["stand_rad"] = round(2 * pra - sta, 6)
            prd, std = target.get("prone_deg"), target.get("stand_deg")
            if prd is not None and std is not None:
                target["stand_deg"] = round(2 * prd - std, 3)
            target["verified"] = False
            msg = f"{name} 方向已取反并存盘（请重新做一次方向验证确认）"
        elif mode == "confirm":
            target["verified"] = True
            msg = f"{name} 方向已确认"
        elif mode == "unconfirm":
            target["verified"] = False
            msg = f"{name} 已取消确认"
        else:
            return False, f"未知方向操作: {mode}"

        try:
            with open(STAND_CONFIG, "w", encoding="utf-8") as f:
                json.dump(cfg, f, ensure_ascii=False, indent=2)
        except OSError as e:
            return False, f"保存失败: {e}"
        with self.lock:
            self.config = cfg
            try:
                self._config_mtime = os.path.getmtime(STAND_CONFIG)
            except OSError:
                self._config_mtime = None
        self.append("[标定] " + msg)
        return True, msg

    # -- 流式伺服进程 --
    def _spawn_servo(self, port, kp, kw):
        cmd = [MOTOR_CTRL, port, "servo", f"{kp}", f"{kw}"]
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

        def reader(p, tag):
            for line in p.stdout:
                s = line.rstrip()
                if s.startswith("FB "):
                    self._update_feedback(s)   # 实时反馈行喂给表格，不进日志
                else:
                    self.append(f"[{tag}] {s}")

        threading.Thread(target=reader, args=(proc, os.path.basename(port)),
                         daemon=True).start()
        return proc

    # 解析 servo 子进程的 "FB id=.. pos=.. vel=.. tau=.. temp=.. merr=.. ok=.. errd=.." 行，
    # 存成每关节实时反馈供前端表格显示；同时用 pos 实时刷新「当前角」。
    def _update_feedback(self, line):
        def g(key):
            m = re.search(rf"\b{key}=(-?\d+(?:\.\d+)?)", line)
            return m.group(1) if m else None
        sid = g("id")
        if sid is None:
            return
        try:
            mid = int(float(sid))
        except ValueError:
            return
        name = self._id2name.get(mid)
        if not name:
            return

        def fnum(key):
            v = g(key)
            return float(v) if v is not None else None

        ok = g("ok") == "1"
        fb = {"vel": fnum("vel"), "tau": fnum("tau"), "errd": fnum("errd"),
              "temp": int(float(g("temp"))) if g("temp") is not None else None,
              "merr": int(float(g("merr"))) if g("merr") is not None else None,
              "ok": ok}
        pos = fnum("pos")
        with self.lock:
            self.feedback[name] = fb
            if pos is not None:                # 保持/运动时也实时刷新当前角
                self.current[name] = pos
                self.current_ok[name] = ok

    def _stop_servos(self, brake=True):
        with self.lock:
            procs = dict(self.servo_procs)
            self.servo_procs = {}
            self.hold_scope = ""      # 电机即将释放，清除「保持中」标记
            self.feedback = {}        # 松开后不再有实时反馈，清空表格反馈列
        for port, proc in procs.items():
            try:
                if brake and proc.poll() is None:
                    proc.stdin.write("stop\n")
                    proc.stdin.flush()
            except (BrokenPipeError, OSError):
                pass
        time.sleep(0.4)  # 给 servo 进程发完 mode=0 停止脉冲的时间
        for port, proc in procs.items():
            if proc.poll() is None:
                try:
                    proc.terminate()
                    proc.wait(timeout=2)
                except Exception:
                    try:
                        proc.kill()
                    except Exception:
                        pass

    # -- 站立执行（安全校验 + 限速插值 + 流式伺服） --
    def _do_stand(self, legs=None, kp=None, kw=None, skip_verify=False):
        try:
            self._stand_impl(legs, kp, kw, skip_verify=skip_verify)
        except Exception as e:
            self.append(f"[异常] 站立执行出错: {e}")
            self.set_status(f"站立异常: {e}")
            self._notify(f"站立执行出错：{e}")
            self._stop_servos()
            with self.lock:
                self.phase = "ERROR"

    def _stand_impl(self, legs=None, kp_override=None, kw_override=None, skip_verify=False):
        cfg = self._load_config()
        if not cfg:
            self.append("[错误] 没有配置文件，请先完成标定并保存。")
            self._notify("没有配置文件，无法执行：请先在标定页完成 ①②③ 并保存。")
            with self.lock:
                self.phase = "IDLE"
            return
        # legs=None 表示整机 12 关节；否则只取这些腿（前缀 fl/fr/rl/rr）
        if legs:
            active = [jt for jt in self.joints
                      if jt["name"].split("_")[0] in legs]
            scope = "单腿测试 [" + ", ".join(sorted(legs)) + "]"
        else:
            active = list(self.joints)
            scope = "整机站立"
        if not active:
            self.append(f"[错误] 没有匹配的关节: {legs}")
            self._notify(f"没有匹配的关节：{legs}")
            with self.lock:
                self.phase = "CONFIGURED"
            return
        ex = cfg.get("execute", self.execute)
        thr = float(ex.get("verify_threshold_rad", 0.20))
        vmax_joint = float(ex.get("max_joint_vel_rad_s", 0.1))
        min_dur = float(ex.get("min_duration_s", 1.0))
        rate = float(ex.get("rate_hz", 100))
        kp = float(ex.get("k_p", 8.0)) if kp_override is None else float(kp_override)
        kw = float(ex.get("k_w", 0.8)) if kw_override is None else float(kw_override)
        if kp_override is not None or kw_override is not None:
            self.append(f"[{scope}] 使用页面指定增益 K_P={kp} K_W={kw}（覆盖配置默认）")

        self.append("\n" + "=" * 56)
        self.append(f"[{scope}] 读取当前角度做安全校验 ...")
        self.set_status(f"{scope}：安全校验中")
        with self.lock:
            self.phase = "EXECUTING"
        res, oks = self._read_all(active)
        with self.lock:
            self.current = dict(res)
            self.current_ok = dict(oks)

        cfg_joints = {j["name"]: j for j in cfg["joints"]}
        cur, prone, delta = {}, {}, {}
        hard_bad = []   # 配置缺失 / 无响应：无法执行，「关闭安全校验」也不能绕过
        dev_bad = []    # 趴姿偏差超阈值：安全校验项，可被「关闭安全校验」跳过
        with self.lock:
            self.last_dev = {}
        for jt in active:
            n = jt["name"]
            cj = cfg_joints.get(n)
            if cj is None:
                hard_bad.append(f"{n}(配置缺失)")
                continue
            if not oks.get(n) or res.get(n) is None:
                hard_bad.append(f"{n}(无响应)")
                continue
            dev = abs(res[n] - cj["prone_rotor"])
            with self.lock:
                self.last_dev[n] = round(dev, 4)
            if dev > thr:
                dev_bad.append(f"{n}(偏差 {dev:.3f} > {thr})")
            cur[n] = res[n]
            prone[n] = cj["prone_rotor"]
            delta[n] = cj["delta_rotor"]

        if hard_bad:
            msg = "无法执行（缺配置 / 无响应）：" + "，".join(hard_bad)
            self.append(f"[拒绝] {msg}")
            self.append("       请确认电机处于电机模式、接线正常，再重试。")
            self.set_status("执行被拒绝（缺配置/无响应）")
            self._notify("⛔ 无法执行：\n\n" + "，".join(hard_bad)
                         + "\n\n请检查电机模式 / 接线后重试。")
            with self.lock:
                self.phase = "CONFIGURED"
            return

        if dev_bad and not skip_verify:
            msg = "安全校验未通过，拒绝执行：" + "，".join(dev_bad)
            self.append(f"[拒绝] {msg}")
            self.append("       请确认机器人已自然趴好、电机处于电机模式，再重试。")
            self.append("       （如确需在非趴姿下测试，可勾选「关闭安全校验」后再点「只测这条腿」。）")
            self.set_status("执行被拒绝（安全校验未通过）")
            self._notify("⛔ 安全校验未通过，已拒绝执行：\n\n" + "，".join(dev_bad)
                         + "\n\n请确认这条腿已自然趴好（各关节贴近标定趴姿）、电机处于电机模式后重试；"
                           "若只想验证电机方向，可改用「方向验证」（不要求趴姿）。")
            with self.lock:
                self.phase = "CONFIGURED"
            return

        if dev_bad and skip_verify:
            self.append("[⚠️ 安全校验已关闭] 跳过趴姿偏差检查，强行按『当前角 + Δ』移动："
                        + "，".join(dev_bad))
            self.append("       危险：若该腿离趴姿较远会大幅运动！请盯住急停。")
            self.set_status(f"{scope}：⚠️ 已关闭安全校验，执行中")
        else:
            self.append(f"[通过] {len(active)} 个关节贴近配置趴姿（阈值 {thr} rad）。")

        # 目标 = 当前 + delta；按关节限速算总时长（位移最大的关节恰好 = vmax）
        targets = {n: cur[n] + delta[n] for n in cur}
        vmax_rotor = vmax_joint * GEAR_RATIO
        max_move = max((abs(targets[n] - cur[n]) for n in cur), default=0.0)
        duration = max(min_dur, max_move / vmax_rotor if vmax_rotor > 0 else min_dur)
        N = max(1, int(duration * rate))
        self.append(f"[轨迹] {scope}：最大位移 {max_move:.3f} rad(转子)，限速 {vmax_joint} rad/s(关节) "
                    f"=> 时长 {duration:.2f}s, {N} 步, K_P={kp} K_W={kw}")
        self.set_status(f"{scope}执行中：缓慢移动（约 {duration:.1f}s）")

        # 每路总线开一个 servo 进程（只开 active 涉及的总线）
        groups = joints_by_port(active)
        with self.lock:
            self.servo_procs = {}
        for port in groups:
            with self.lock:
                self.servo_procs[port] = self._spawn_servo(port, kp, kw)
        time.sleep(0.2)  # 等 servo 进程起来（含 sudo 鉴权）
        with self.lock:
            self.hold_scope = scope   # 标记电机已锁位保持，供「是否松开」显示

        dt = 1.0 / rate
        for k in range(1, N + 1):
            if self.abort.is_set():
                self.append("[中止] 收到急停/中止，停止插值。")
                self._stop_servos()
                with self.lock:
                    self.phase = "CONFIGURED"
                self.set_status("站立已中止")
                return
            a = k / N  # 0->1 线性插值
            for port, jts in groups.items():
                parts = []
                for jt in jts:
                    n = jt["name"]
                    if n not in cur:
                        continue
                    pos = cur[n] + a * (targets[n] - cur[n])
                    parts.append(f"{jt['id']} {pos:.5f}")
                proc = self.servo_procs.get(port)
                if proc and proc.poll() is None and parts:
                    try:
                        proc.stdin.write(" ".join(parts) + "\n")
                        proc.stdin.flush()
                    except (BrokenPipeError, OSError):
                        self.append(f"[警告] servo {port} 管道已断开")
            time.sleep(dt)

        self.append(f"[完成] {scope}到位，servo 进程持续保持位置（点「急停/松开」释放）。")
        if legs:
            self.append("       请检查这条腿：是否朝『站起来』方向收拢？哪个关节方向不对就反它的标定。")
        self.set_status(f"{scope}到位（保持中）")
        with self.lock:
            self.phase = "STANDING"

    # -- 方向验证：单腿，每个关节从当前位置朝「站立方向」各转固定角度（默认 10°）--
    # 与站立不同：不要求处于趴姿（不做 prone 偏差校验），只做小幅相对位移验证方向符号。
    def _do_dir_test(self, leg, deg=10.0, joint=None, kp=None, kw=None):
        try:
            self._dir_test_impl(leg, deg, joint, kp, kw)
        except Exception as e:
            self.append(f"[异常] 方向验证出错: {e}")
            self.set_status(f"方向验证异常: {e}")
            self._notify(f"方向验证出错：{e}")
            self._stop_servos()
            with self.lock:
                self.phase = "ERROR"

    def _dir_test_impl(self, leg, deg=10.0, joint=None, kp_override=None, kw_override=None):
        cfg = self._load_config()
        if not cfg:
            self.append("[错误] 没有配置文件，请先完成标定并保存。")
            self._notify("没有配置文件，无法验证：请先在标定页完成 ①②③ 并保存。")
            with self.lock:
                self.phase = "IDLE"
            return
        ex = cfg.get("execute", self.execute)
        vmax_joint = float(ex.get("max_joint_vel_rad_s", 0.05))
        min_dur = float(ex.get("min_duration_s", 2.0))
        rate = float(ex.get("rate_hz", 100))
        kp = float(ex.get("k_p", 2.0)) if kp_override is None else float(kp_override)
        kw = float(ex.get("k_w", 1.0)) if kw_override is None else float(kw_override)
        if kp_override is not None or kw_override is not None:
            self.append(f"[方向验证] 使用页面指定增益 K_P={kp} K_W={kw}（覆盖配置默认）")

        if joint in ("hip", "thigh", "shank"):
            active = [jt for jt in self.joints if jt["name"] == f"{leg}_{joint}"]
            scope = f"方向验证 [{leg}_{joint}] 单电机 +{deg:.0f}°"
        else:
            active = [jt for jt in self.joints if jt["name"].split("_")[0] == leg]
            scope = f"方向验证 [{leg}] 各关节 +{deg:.0f}°"
        if not active:
            self.append(f"[错误] 没有匹配的关节: {leg}_{joint or '*'}")
            self._notify(f"没有匹配的关节：{leg}_{joint or '*'}")
            with self.lock:
                self.phase = "CONFIGURED"
            return
        port = active[0]["port"]
        cfg_joints = {j["name"]: j for j in cfg["joints"]}

        self.append("\n" + "=" * 56)
        self.append(f"[{scope}] 读取当前角度（不要求趴姿）...")
        self.set_status(f"{scope}：读取中")
        with self.lock:
            self.phase = "EXECUTING"
        res, oks = self._read_all(active)
        with self.lock:
            self.current = dict(res)
            self.current_ok = dict(oks)

        step_rotor = math.radians(deg) * GEAR_RATIO   # 关节角 deg -> 转子 rad
        cur, targets, plan, bad = {}, {}, [], []
        for jt in active:
            n = jt["name"]
            if not oks.get(n) or res.get(n) is None:
                bad.append(f"{n}(无响应)")
                continue
            cj = cfg_joints.get(n, {})
            dr = cj.get("delta_rotor")
            d = cj.get("dir")
            sign = d if d in (1, -1) else (1 if (dr or 0) > 0 else (-1 if (dr or 0) < 0 else 0))
            if sign == 0:
                self.append(f"  [{n}] 配置方向为 0/缺失，跳过（趴↔站几乎不动）")
                continue
            cur[n] = res[n]
            targets[n] = res[n] + sign * step_rotor
            plan.append((n, jt["id"], sign))

        if bad:
            self.append(f"[拒绝] 有关节无响应：{'，'.join(bad)}；检查接线/电机模式后重试。")
            self.set_status("方向验证被拒绝（有关节无响应）")
            self._notify("⛔ 方向验证被拒绝：有关节无响应：\n\n" + "，".join(bad)
                         + "\n\n请检查该腿接线、供电，并确认电机已进入电机模式后重试。")
            with self.lock:
                self.phase = "CONFIGURED"
            return
        if not targets:
            self.append("[结束] 没有可验证的关节（方向都为 0）。")
            self._notify("没有可验证的关节：所选关节在配置里方向都为 0（趴↔站几乎不动），无法验证方向。",
                         level="warn")
            with self.lock:
                self.phase = "CONFIGURED"
            return

        for n, i, sign in plan:
            self.append(f"  {n}: 朝站立方向 {'＋' if sign > 0 else '－'} 转 {deg:.0f}°(关节)")

        vmax_rotor = vmax_joint * GEAR_RATIO
        duration = max(min_dur, step_rotor / vmax_rotor if vmax_rotor > 0 else min_dur)
        N = max(1, int(duration * rate))
        self.append(f"[轨迹] {scope}：每关节 {step_rotor:.3f} rad(转子) => 时长 {duration:.2f}s, "
                    f"{N} 步, K_P={kp} K_W={kw}")
        self.set_status(f"{scope}执行中（约 {duration:.1f}s）")

        servo = self._spawn_servo(port, kp, kw)
        with self.lock:
            self.servo_procs = {port: servo}
            self.hold_scope = scope   # 标记电机已锁位保持，供「是否松开」显示
        time.sleep(0.2)

        dt = 1.0 / rate
        for k in range(1, N + 1):
            if self.abort.is_set():
                self.append("[中止] 收到急停/中止。")
                self._stop_servos()
                with self.lock:
                    self.phase = "CONFIGURED"
                self.set_status("方向验证已中止")
                return
            a = k / N
            parts = []
            for n, i, sign in plan:
                pos = cur[n] + a * (targets[n] - cur[n])
                parts.append(f"{i} {pos:.5f}")
            if servo.poll() is None and parts:
                try:
                    servo.stdin.write(" ".join(parts) + "\n")
                    servo.stdin.flush()
                except (BrokenPipeError, OSError):
                    self.append(f"[警告] servo {port} 管道已断开")
            time.sleep(dt)

        self.append(f"[完成] {scope}到位，servo 保持中。请核对：每个关节是否朝『站起来』方向转了？")
        self.append("       哪个关节往反方向转 = 那个关节标定方向标反了。点「急停/松开」释放。")
        self.set_status(f"{scope}到位（保持中）")
        with self.lock:
            self.phase = "STANDING"

    # -- 急停 / 释放 --
    def _estop(self):
        self.abort.set()
        self.append("\n[急停] 中止动作并释放电机 ...")
        self.set_status("急停")
        self._stop_servos()
        with self.lock:
            self._refresh_phase()
        self.append("[急停] 已停止。")

    def _finish(self, title):
        with self.lock:
            self.busy = False
        self.set_status(f"完成: {title}")

    def _start_bg(self, fn, *args):
        with self.lock:
            self.busy = True
        self.abort.clear()
        threading.Thread(target=fn, args=args, daemon=True).start()

    # -- 入口 --
    def run(self, action, p):
        with self.lock:
            busy = self.busy
            standing = (self.standing_thread is not None
                        and self.standing_thread.is_alive())

        if action == "estop":
            threading.Thread(target=self._estop, daemon=True).start()
            return True, "ok"

        if action == "read":
            if busy or standing:
                return False, "忙：请等待当前操作完成"
            self._start_bg(self._read_current)
            return True, "ok"

        if action == "reload_config":
            return self._reload_config_now()

        if action in ("calib_prone", "calib_stand"):
            if busy or standing:
                return False, "忙：请等待当前操作完成"
            self._start_bg(self._calib, "prone" if action == "calib_prone" else "stand")
            return True, "ok"

        if action == "save_config":
            if busy or standing:
                return False, "忙：请等待当前操作完成"
            ok, msg = self._save_config()
            return ok, msg

        if action == "stand":
            if busy:
                return False, "忙：请等待当前操作完成"
            if standing:
                return False, "已在执行站立"
            if self.config is None:
                return False, "尚未标定保存配置，无法站立"
            self.abort.clear()
            t = threading.Thread(target=self._do_stand, daemon=True)
            with self.lock:
                self.standing_thread = t
            t.start()
            return True, "ok"

        if action == "test_leg":
            if busy:
                return False, "忙：请等待当前操作完成"
            if standing:
                return False, "已在执行动作"
            if self.config is None:
                return False, "尚未标定保存配置，无法测试"
            leg = str(p.get("leg", "")).strip().lower()
            if leg not in ("fl", "fr", "rl", "rr"):
                return False, "腿名必须是 fl/fr/rl/rr"
            kp = parse_opt_gain(p.get("kp", ""))
            kw = parse_opt_gain(p.get("kw", ""))
            if kp == "ERR" or kw == "ERR":
                return False, "K_P / K_W 必须是数字（留空表示用配置默认值）"
            skip_verify = str(p.get("skip_verify", "")).strip() in ("1", "true", "True", "on")
            if skip_verify:
                self.append("[请求] 「只测这条腿」已勾选『关闭安全校验』，将跳过趴姿偏差检查。")
            self.abort.clear()
            t = threading.Thread(target=self._do_stand, args=({leg},),
                                 kwargs={"kp": kp, "kw": kw, "skip_verify": skip_verify},
                                 daemon=True)
            with self.lock:
                self.standing_thread = t
            t.start()
            return True, "ok"

        if action == "dir_test":
            if busy:
                return False, "忙：请等待当前操作完成"
            if standing:
                return False, "已在执行动作"
            if self.config is None:
                return False, "尚未标定保存配置，无法验证"
            leg = str(p.get("leg", "")).strip().lower()
            if leg not in ("fl", "fr", "rl", "rr"):
                return False, "腿名必须是 fl/fr/rl/rr"
            joint = str(p.get("joint", "")).strip().lower() or None
            if joint not in (None, "hip", "thigh", "shank", "all"):
                return False, "关节必须是 hip/thigh/shank（或 all/留空表示整腿）"
            if joint == "all":
                joint = None
            try:
                deg = float(p.get("deg", 10.0))
            except (TypeError, ValueError):
                deg = 10.0
            deg = max(1.0, min(30.0, deg))   # 限幅 1~30°，防误填大角度
            kp = parse_opt_gain(p.get("kp", ""))
            kw = parse_opt_gain(p.get("kw", ""))
            if kp == "ERR" or kw == "ERR":
                return False, "K_P / K_W 必须是数字（留空表示用配置默认值）"
            self.abort.clear()
            t = threading.Thread(target=self._do_dir_test, args=(leg, deg, joint),
                                 kwargs={"kp": kp, "kw": kw}, daemon=True)
            with self.lock:
                self.standing_thread = t
            t.start()
            return True, "ok"

        if action in ("confirm_dir", "unconfirm_dir", "invert_dir"):
            if busy or standing:
                return False, "忙：请等待当前动作完成再确认/取反"
            if self.config is None:
                return False, "尚无配置文件"
            name = str(p.get("name", "")).strip()
            if not name:
                return False, "缺少关节名"
            mode = {"confirm_dir": "confirm", "unconfirm_dir": "unconfirm",
                    "invert_dir": "invert"}[action]
            return self._set_dir(name, mode)

        return False, f"未知操作: {action}"

    def _read_current(self):
        self.append("\n[读取] 读取全部 12 关节当前转子角 ...")
        self.set_status("读取当前角度")
        res, oks = self._read_all()
        with self.lock:
            self.current = dict(res)
            self.current_ok = dict(oks)
        self._finish("读取角度")


CTRL = RobotController()


def check_env():
    if not os.path.isfile(MOTOR_CTRL):
        CTRL.append(f"[警告] 未找到 {MOTOR_CTRL}；请在 SDK 目录 build/ 执行 cmake .. && make motor_ctrl")
    else:
        CTRL.append(f"[就绪] motor_ctrl: {MOTOR_CTRL}")
    if CTRL.config:
        CTRL.append(f"[就绪] 已加载配置 {STAND_CONFIG}（标定于 {CTRL.config.get('calibrated_at','?')}）")
        CTRL.append("[提示] 单圈绝对值编码器，绝对位置断电不变，配置跨上电有效；仅在机械改动/重新装配/换电机后才需重标。")
    else:
        CTRL.append("[提示] 尚无配置文件，请先按 ①②③ 完成标定。")


# ----------------------------------------------------------------- 前端页面
PAGE = r"""<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>四足机器人操控</title>
<style>
  :root { --bd:#d0d4da; --fg:#222; --muted:#888; --warn:#a60; --pri:#1769d6; --ok:#2a8a3e; --bad:#d64545; }
  * { box-sizing: border-box; }
  body { margin:0; padding:16px; color:var(--fg); background:#f5f6f8;
    font-family: system-ui, -apple-system, "Noto Sans CJK SC", "Microsoft YaHei", sans-serif; }
  h1 { font-size:18px; margin:0 0 12px; }
  fieldset { border:1px solid var(--bd); border-radius:8px; margin:0 0 12px; padding:10px 12px; background:#fff; }
  legend { font-weight:600; padding:0 6px; color:#555; }
  .row { display:flex; flex-wrap:wrap; align-items:center; gap:8px 10px; }
  button { padding:7px 13px; border:1px solid var(--bd); border-radius:6px; background:#fff; cursor:pointer; font-size:14px; }
  button:hover:not(:disabled){ border-color:var(--pri); color:var(--pri); }
  button:disabled{ opacity:.45; cursor:not-allowed; }
  button.primary{ background:var(--pri); color:#fff; border-color:var(--pri); }
  button.stand{ background:var(--ok); color:#fff; border-color:var(--ok); font-size:16px; padding:12px 26px; }
  button.estop{ background:var(--bad); color:#fff; border-color:var(--bad); font-weight:700; }
  button.mini{ padding:2px 7px; font-size:12px; }
  .pill{ font-size:12px; padding:3px 10px; border-radius:11px; background:#eee; color:#555; }
  .pill.ok{ background:#e3f4e7; color:var(--ok); }
  .pill.warn{ background:#fff8e1; color:var(--warn); }
  table{ border-collapse:collapse; width:100%; font-size:13px; }
  th,td{ border:1px solid var(--bd); padding:4px 7px; text-align:center; }
  th{ background:#eef1f5; }
  td.bad{ background:#fde8e8; color:var(--bad); font-weight:600; }
  .pad{ display:grid; grid-template-columns:repeat(3,52px); grid-template-rows:repeat(3,52px); gap:6px; }
  .pad button{ width:52px; height:52px; padding:0; font-size:18px; }
  .grow{ flex:1; }
  pre#log{ height:240px; overflow:auto; margin:0; padding:8px 10px; background:#1e1e1e; color:#e0e0e0;
    border-radius:6px; font-family:"Noto Sans Mono CJK SC", ui-monospace, monospace; font-size:12.5px;
    line-height:1.45; white-space:pre-wrap; word-break:break-all; }
  #status{ flex:1; padding:6px 10px; background:#fff; border:1px solid var(--bd); border-radius:6px; }
  .banner{ display:none; margin:0 0 12px; padding:8px 12px; border-radius:8px; font-size:13px; }
  .banner.bad{ display:block; border:1px solid var(--bad); background:#fde8e8; color:var(--bad); }
  .banner.hold{ display:block; border:1px solid var(--warn); background:#fff8e1; color:var(--warn); }
  /* 右上角固定面板：驱动状态 + 大急停按钮（纵向堆叠，占右上空白区，不占整行）*/
  #driveBar{ position:fixed; top:12px; right:14px; z-index:1000; width:256px;
    display:flex; flex-direction:column; align-items:stretch; gap:8px;
    padding:10px; background:rgba(255,255,255,.96); border:1px solid var(--bd); border-radius:10px;
    box-shadow:0 2px 12px rgba(0,0,0,.18); transition:background .15s ease, border-color .15s ease; }
  #driveBar.on{ background:#fde8e8; border-color:#d64545; }
  #driveState{ text-align:center; font-size:28px; font-weight:700; padding:10px 12px; border-radius:9px;
    white-space:nowrap; background:#e3f4e7; color:#2a8a3e; }
  #driveBar.on #driveState{ background:#d64545; color:#fff; animation:dpulse 1s steps(1,end) infinite; }
  @keyframes dpulse{ 50%{ opacity:.45; } }
  /* 图标（● / ⏹）单独放大：相对各自所在文字再 2 倍 */
  .bigico{ font-size:2em; vertical-align:middle; }
  #btnEstop{ width:100%; background:#d64545; color:#fff; border:2px solid #fff; border-radius:8px;
    font-size:36px; font-weight:800; letter-spacing:1px; line-height:1.25; padding:16px 8px; cursor:pointer;
    box-shadow:0 1px 4px rgba(0,0,0,.25); }
  #btnEstop:hover{ background:#bf3a3a; color:#fff; border-color:#fff; }
  /* 驱动中：整页红色边框警示（不挡点击）*/
  #driveEdge{ position:fixed; inset:0; border:6px solid #d64545; pointer-events:none; z-index:999; display:none; }
  #driveEdge.on{ display:block; animation:dpulse 1s steps(1,end) infinite; }
</style>
</head>
<body>
  <div id="driveBar">
    <span id="driveState"><span class="bigico">●</span> 未驱动</span>
    <button id="btnEstop" type="button" onclick="estopNow()"><span class="bigico">⏹</span> 急停<br>STOP</button>
  </div>
  <div id="driveEdge"></div>
  <h1>🐕 四足机器人操控 <span class="muted" style="font-size:13px">（站立标定 / 执行）</span></h1>

  <div id="warnBanner" class="banner bad"></div>
  <div id="holdBanner" class="banner"></div>

  <fieldset>
    <legend>状态</legend>
    <div class="row">
      <span class="pill" id="phasePill">阶段: -</span>
      <span class="pill" id="cfgPill">配置: -</span>
      <span class="pill" id="holdPill" title="电机是否仍带电锁位保持（未松开）">松开状态: -</span>
      <button onclick="api('reload_config')" title="从磁盘重新读取 config/stand_config.json（在 motor_web 标定保存后用它刷新趴姿/站姿）">📂 重新加载配置</button>
      <button class="primary" onclick="api('read')">📐 读取当前 12 关节角</button>
      <span class="grow"></span>
      <button class="estop" onclick="api('estop')">⏹ 急停 / 松开</button>
    </div>
  </fieldset>

  <fieldset>
    <legend>站立标定（单圈绝对值编码器：标定一次跨上电有效，机械改动后再重标）</legend>
    <div class="row">
      <button onclick="api('calib_prone')">① 记录趴姿</button>
      <button onclick="api('calib_stand')">② 记录站姿（手扶撑起后点）</button>
      <button onclick="confirmSave()">③ 计算并保存配置</button>
      <span class="pill" id="pronePill">趴姿: 未记录</span>
      <span class="pill" id="standPill">站姿: 未记录</span>
    </div>
    <p class="muted" style="font-size:12.5px;margin:8px 0 0">
      流程：让机器人自然趴下点①；手扶把它撑成站立姿态点②；点③算出每关节趴→站的转动量(Δ)与方向并存盘。
    </p>
  </fieldset>

  <fieldset>
    <legend>操控（手柄）</legend>
    <div class="row" style="align-items:flex-start; gap:28px">
      <div class="pad">
        <span></span><button disabled title="后续">▲</button><span></span>
        <button disabled title="后续">◀</button><button disabled title="后续">●</button><button disabled title="后续">▶</button>
        <span></span><button disabled title="后续">▼</button><span></span>
      </div>
      <div>
        <button class="stand" id="btnStand" onclick="confirmStand()">🧍 站立</button>
        <button class="stand" style="background:#777;border-color:#777" disabled title="后续">🪑 坐下</button>
        <p class="muted" style="font-size:12.5px;margin:8px 0 0;max-width:360px">
          点「站立」会先读当前角并与配置趴姿比对（阈值见下表），全部贴近才放行，然后按关节限速
          (≤0.1 rad/s) 缓慢起身。方向键为后续行走动作占位。
        </p>
      </div>
    </div>
    <div class="row" style="margin-top:12px; border-top:1px dashed var(--bd); padding-top:10px">
      <label>单腿验证</label>
      <select id="legSel">
        <option value="fl">左前 FL (ttyUSB0 / id 1,2,3)</option>
        <option value="fr">右前 FR (ttyUSB1 / id 4,5,6)</option>
        <option value="rl">左后 RL (ttyUSB2 / id 7,8,9)</option>
        <option value="rr">右后 RR (ttyUSB3 / id 10,11,12)</option>
      </select>
      <label title="位置环刚度。留空=用配置 execute.k_p；越大越硬、力矩越大。范围 0~25.599">K_P</label>
      <input id="legKp" type="number" min="0" max="25.599" step="0.1" placeholder="默认" style="width:80px; padding:5px 7px; border:1px solid var(--bd); border-radius:6px; font-size:14px">
      <label title="速度阻尼。留空=用配置 execute.k_w。范围 0~25.599">K_W</label>
      <input id="legKw" type="number" min="0" max="25.599" step="0.1" placeholder="默认" style="width:80px; padding:5px 7px; border:1px solid var(--bd); border-radius:6px; font-size:14px">
      <button id="btnTestLeg" onclick="confirmTestLeg()">🦵 只测这条腿（趴→站）</button>
      <label id="skipWrap" title="勾选后『只测这条腿』将跳过『当前是否≈趴姿』的安全校验，直接按 当前角+Δ 运动。危险：若不在趴姿附近会大幅运动！" style="color:#c0392b; font-size:12.5px; user-select:none">
        <input type="checkbox" id="legSkipVerify" style="vertical-align:middle"> 关闭安全校验
      </label>
      <span style="width:10px"></span>
      <label>电机</label>
      <select id="dirJoint">
        <option value="hip">hip 髋</option>
        <option value="thigh">thigh 大腿</option>
        <option value="shank">shank 小腿</option>
        <option value="all">全部(3个)</option>
      </select>
      <label>角度°</label>
      <input id="dirDeg" type="number" min="1" max="30" step="1" value="10" style="width:64px; padding:5px 7px; border:1px solid var(--bd); border-radius:6px; font-size:14px">
      <button id="btnDirTest" onclick="confirmDirTest()">🧭 方向验证（单电机转设定角度）</button>
      <span class="muted" style="font-size:12.5px">只动选中那条腿、只在它那一路总线发指令，其它腿不碰。请机器人架空、该腿周围清空。</span>
    </div>
    <p class="muted" style="font-size:12.5px;margin:8px 0 0">
      「趴→站」从趴姿走到站姿（要求当前≈趴姿）；「方向验证」不要求趴姿——选一个电机(hip/thigh/shank)，从当前位置朝配置里的站立方向转设定角度，**一次只驱一个电机**、专门核对该电机方向符号对不对。
      上面的 <b>K_P / K_W</b> 同时作用于这两个单腿动作：留空则各自沿用配置 <code>execute</code> 里的默认值（输入框灰字即当前默认值）；填了就临时覆盖（不写回配置文件）。调小 K_P 更软更安全，调大更硬跟随更紧。
    </p>
  </fieldset>

  <fieldset>
    <legend>关节角（单位：度°，= 转子角 ÷ 6.33 × 180/π；内部控制/校验仍用转子角 rad）</legend>
    <table>
      <thead><tr>
        <th>关节</th><th>串口</th><th>ID</th><th>当前(°)</th><th>趴姿(°)</th><th>站姿(°)</th>
        <th>Δ(站−趴,°)</th><th>方向</th><th>校验偏差(°)</th>
        <th title="实测转子角速度，保持时应≈0；来回跳动=震动">实测ω(rad/s)</th>
        <th title="实测力矩(N·m)，保持时应为较小常值；大幅摆动=震动">实测τ(N·m)</th>
        <th title="目标−实测 的跟踪误差(关节°)，保持/运动时应趋近 0">跟踪误差(°)</th>
        <th>确认/取反</th>
      </tr></thead>
      <tbody id="jbody"><tr><td colspan="13" class="muted">（点「读取当前 12 关节角」）</td></tr></tbody>
    </table>
  </fieldset>

  <fieldset>
    <legend>输出</legend>
    <pre id="log"></pre>
  </fieldset>

  <div class="row"><span id="status">就绪</span></div>

<script>
  let logIndex = 0;
  let lastNoticeId = null;   // 已弹过的 notice id；null=尚未建立基线（首次轮询只记基线，不回放旧通知）
  let lastExecute = null;    // 最近一次 poll 拿到的 execute 配置（含 K_P/K_W 默认值），供 placeholder / 确认框显示
  async function api(action, params){
    params = Object.assign({action}, params||{});
    try{
      const r = await fetch('/api/run',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(params)});
      const j = await r.json();
      if(!j.ok) alert(j.message||'操作失败');
    }catch(e){ alert('请求失败: '+e); }
  }
  function estopNow(){ api('estop'); }   // 急停：立即中止动作并松开电机，无需确认
  function confirmSave(){
    if(confirm('将根据已记录的趴姿/站姿计算各关节Δ并写入 stand_config.json？')) api('save_config');
  }
  function confirmStand(){
    if(confirm('确认执行站立？\n\n请确保：机器人已自然趴好、周围无人无障碍、可随时按急停。')) api('stand');
  }
  // 读单腿验证用的 K_P / K_W：留空则不传，后端沿用配置默认值
  function legGains(){
    const g = {};
    const kp = document.getElementById('legKp').value.trim();
    const kw = document.getElementById('legKw').value.trim();
    if (kp !== '') g.kp = kp;
    if (kw !== '') g.kw = kw;
    return g;
  }
  function gainNote(){
    const g = legGains();
    const dk = (lastExecute && gfmt(lastExecute.k_p)) || '配置';
    const dw = (lastExecute && gfmt(lastExecute.k_w)) || '配置';
    if (g.kp === undefined && g.kw === undefined)
      return '增益：用配置默认值（K_P=' + dk + ' K_W=' + dw + '）。';
    return '增益：K_P=' + (g.kp ?? ('默认'+dk)) + '  K_W=' + (g.kw ?? ('默认'+dw)) + '（覆盖配置）。';
  }
  function confirmTestLeg(){
    const leg=document.getElementById('legSel').value;
    const skip=document.getElementById('legSkipVerify').checked;
    let msg='只测试 '+leg.toUpperCase()+' 这一条腿（趴→站），其它腿不动。\n'+gainNote();
    if(skip) msg+='\n\n⚠️⚠️ 已【关闭安全校验】：不检查是否在趴姿，直接按 当前角+Δ 运动。\n若该腿当前离趴姿较远，会发生大幅、可能危险的运动！';
    msg+='\n\n请确保：机器人已架空、该腿周围无障碍、可随时按急停。';
    if(confirm(msg)) api('test_leg', Object.assign({leg, skip_verify: skip?1:0}, legGains()));
  }
  function confirmInvert(name){
    if(confirm('取反 '+name+' 的标定方向？\n\n会翻转该电机的 delta_rotor / dir 并写回 stand_config.json。建议取反后再做一次「方向验证」确认。')) api('invert_dir',{name});
  }
  function confirmDirTest(){
    const leg=document.getElementById('legSel').value;
    const joint=document.getElementById('dirJoint').value;
    let deg=parseFloat(document.getElementById('dirDeg').value);
    if(isNaN(deg)||deg<1||deg>30){ alert('角度需在 1~30° 之间'); return; }
    const tgt = (joint==='all') ? (leg.toUpperCase()+' 全部 3 个电机') : (leg.toUpperCase()+'_'+joint+' 这一个电机');
    if(confirm('方向验证：只驱动 '+tgt+'，从当前位置朝『站立方向』转约 '+deg+'°（不要求趴姿），其它电机不动。\n'+gainNote()+'\n\n请确保：机器人已架空、该腿周围无障碍、可随时按急停。')) api('dir_test', Object.assign({leg, joint, deg}, legGains()));
  }
  const f=(v,n)=>{ if(v===null||v===undefined) return '—'; const x=parseFloat(v); return isNaN(x)?'—':x.toFixed(n); };
  // 显示用：转子角(rad) -> 关节角(度)。仅换显示单位；校验偏差标红、方向符号等内部判断仍按转子角 rad。
  const jdeg=(v,gear)=>{ if(v===null||v===undefined) return '—'; const x=parseFloat(v); return isNaN(x)?'—':(x/(gear||6.33)*180/Math.PI).toFixed(2); };
  // 增益(K_P/K_W)显示：整数补一位小数(2 -> 2.0)，其余原样；非数字返回 null
  const gfmt=(v)=>{ const x=parseFloat(v); return isNaN(x)?null:(Number.isInteger(x)?x.toFixed(1):String(x)); };

  function render(s){
    document.getElementById('phasePill').textContent = '阶段: ' + s.phase;
    const driving = !!(s.holding || s.standing);
    document.getElementById('driveState').innerHTML = driving ? '<span class="bigico">●</span> 驱动中' : '<span class="bigico">●</span> 未驱动';
    document.getElementById('driveBar').classList.toggle('on', driving);
    document.getElementById('driveEdge').classList.toggle('on', driving);
    const cp=document.getElementById('cfgPill');
    cp.textContent = '配置: ' + (s.configured?'已标定':'未标定');
    cp.className = 'pill ' + (s.configured?'ok':'warn');
    document.getElementById('pronePill').textContent = '趴姿: ' + (s.has_prone?'已记录':'未记录');
    document.getElementById('standPill').textContent = '站姿: ' + (s.has_stand?'已记录':'未记录');
    document.getElementById('btnStand').disabled = !s.configured || s.busy || s.standing;
    document.getElementById('btnTestLeg').disabled = !s.configured || s.busy || s.standing;
    document.getElementById('btnDirTest').disabled = !s.configured || s.busy || s.standing;
    document.getElementById('status').textContent = s.status;

    // K_P/K_W 留空时实际生效的是配置 execute 里的默认值；把默认值显示到输入框灰字 placeholder
    if (s.execute){
      lastExecute = s.execute;
      const pk=gfmt(s.execute.k_p), pw=gfmt(s.execute.k_w);
      document.getElementById('legKp').placeholder = (pk!==null) ? ('默认 '+pk) : '配置';
      document.getElementById('legKw').placeholder = (pw!==null) ? ('默认 '+pw) : '配置';
    }

    // 是否松开：后端按 servo 进程是否存活判定（holding=电机仍带电锁位）
    const holding = !!s.holding;
    const hp=document.getElementById('holdPill');
    if (holding && s.standing){ hp.textContent='松开状态: 🔒 电机运行中'; hp.className='pill warn'; }
    else if (holding){ hp.textContent='松开状态: 🔒 保持锁位（未松开）'; hp.className='pill warn'; }
    else { hp.textContent='松开状态: 🔓 已松开'; hp.className='pill ok'; }
    hp.title = holding ? ('电机仍带电锁位'+(s.hold_scope?('：'+s.hold_scope):'')+'；点「急停/松开」释放')
                       : '电机自由、未上电锁位';
    // 动作到位但电机仍锁位（未松开）时，给一条醒目横幅 + 就地「松开」按钮
    const hb=document.getElementById('holdBanner');
    if (holding && !s.standing){
      hb.className='banner hold';
      hb.innerHTML='🔒 动作已到位，电机<b>仍在锁位保持</b>'+(s.hold_scope?('（'+s.hold_scope+'）'):'')
        +'，<b>尚未松开</b>。检查完毕后请点 '
        +'<button class="mini" style="margin-left:6px" onclick="api(\'estop\')">🔓 松开电机</button>';
    } else {
      hb.className='banner'; hb.innerHTML='';
    }

    const thr = (s.execute && s.execute.verify_threshold_rad) || 0.2;  // 转子角 rad，与 j.dev 同坐标，仅用于标红
    const gear = s.gear_ratio || 6.33;
    const tb=document.getElementById('jbody');
    if(!s.joints||!s.joints.length){ } else {
      tb.innerHTML='';
      for(const j of s.joints){
        const tr=document.createElement('tr');
        const devBad = (j.dev!==null&&j.dev!==undefined&&j.dev>thr);
        const dir = (j.delta===null||j.delta===undefined)?'—':(j.delta>=0?'↑ +':'↓ −');
        const okMark = (j.ok===false)?' <span style="color:var(--bad)">⚠</span>':'';
        tr.innerHTML =
          '<td>'+j.name+'</td><td>'+j.port.replace('/dev/','')+'</td><td>'+j.id+'</td>'+
          '<td>'+jdeg(j.current,gear)+okMark+'</td><td>'+jdeg(j.prone,gear)+'</td><td>'+jdeg(j.stand,gear)+'</td>'+
          '<td>'+(j.delta>=0?'+':'')+jdeg(j.delta,gear)+'</td><td>'+dir+'</td>'+
          '<td class="'+(devBad?'bad':'')+'">'+jdeg(j.dev,gear)+'</td>'+
          '<td>'+f(j.fb_vel,3)+'</td><td>'+f(j.fb_tau,3)+'</td><td>'+f(j.fb_errd,2)+'</td>'+
          '<td>'+
          (j.verified
            ? '<span style="color:var(--ok)">✅</span> <button class="mini" title="撤销确认，恢复待验证" onclick="api(\'unconfirm_dir\',{name:\''+j.name+'\'})">↩取消确认</button>'
            : '<button class="mini" title="方向正确，标记已确认" onclick="api(\'confirm_dir\',{name:\''+j.name+'\'})">✓确认</button>')+
          ' <button class="mini" title="方向反了，翻转该电机方向并存盘" onclick="confirmInvert(\''+j.name+'\')">⇄取反</button>'+
          '</td>';
        tb.appendChild(tr);
      }
    }
    const wb=document.getElementById('warnBanner');
    if(s.phase==='ERROR'){ wb.className='banner bad'; wb.textContent='⚠ 上次站立执行出错，请查看日志并急停后重试。'; }
    else { wb.className='banner'; wb.textContent=''; }
  }

  async function poll(){
    try{
      const r=await fetch('/api/state?since='+logIndex);
      const s=await r.json();
      const log=document.getElementById('log');
      if(s.log&&s.log.length){
        const atBottom = log.scrollTop+log.clientHeight >= log.scrollHeight-24;
        log.textContent += s.log.join('\n')+'\n';
        if(atBottom) log.scrollTop=log.scrollHeight;
      }
      logIndex=s.log_index;
      // 后台线程的拒绝/异常通过 notice 通道弹窗（首次轮询只记基线，避免回放旧通知）
      const nid = s.notice ? s.notice.id : 0;
      if (lastNoticeId === null) lastNoticeId = nid;
      else if (s.notice && nid !== lastNoticeId){ lastNoticeId = nid; alert(s.notice.text); }
      render(s);
    }catch(e){}
  }
  poll();
  setInterval(poll, 400);
</script>
</body>
</html>
"""


# ----------------------------------------------------------------- HTTP
class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _send(self, data, ctype):
        self.send_response(200)
        self.send_header("Content-Type", ctype)
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
        elif u.path == "/api/state":
            q = parse_qs(u.query)
            try:
                since = int(q.get("since", ["0"])[0])
            except (ValueError, TypeError):
                since = 0
            self._json(CTRL.snapshot(since))
        elif u.path == "/api/config":
            self._json(CTRL.config or {})
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
    port = int(os.environ.get("PORT", "8001"))
    check_env()
    srv = ThreadingHTTPServer((host, port), Handler)
    print(f"四足操控网页已启动:  http://{host}:{port}")
    if host not in ("127.0.0.1", "localhost"):
        print("⚠️  正在监听非本机地址，局域网内任何人都能操控机器人，请注意安全。")
    print("按 Ctrl+C 退出。")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\n正在停止 ...")
        CTRL._estop()
    finally:
        srv.server_close()


if __name__ == "__main__":
    main()
