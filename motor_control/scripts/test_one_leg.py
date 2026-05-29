#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""单腿标定验证 —— 只驱动一条腿，从趴姿缓慢插值到站姿，用于安全地核对方向。

为什么单独写：整机站立(robot_web.py)会同时驱动 12 个关节、在 4 路总线上一起发力，
首次验证风险大。本脚本只在**一条腿对应的那一路串口**上开 servo，只发那条腿
3 个关节的目标，其它腿完全不碰；动作前先打印计划让你确认，全程慢速低增益，
可随时 Ctrl-C 释放电机。

用法（建议机器人架空、目标腿周围清空）：
    python3 scripts/test_one_leg.py            # 默认左前腿 fl
    python3 scripts/test_one_leg.py fr         # 右前腿
    python3 scripts/test_one_leg.py rl|rr      # 左后/右后

读取 config/stand_config.json 里该腿各关节的 prone_rotor / delta_rotor / execute 参数。
流程：读当前角 -> 安全校验(当前≈趴姿) -> 打印计划并请你确认 -> 慢速插值到站姿 ->
保持位置，等你按回车(或 Ctrl-C) -> 发停止脉冲释放。
"""

import json
import os
import re
import subprocess
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
_DEFAULT_SDK_ROOT = os.path.normpath(os.path.join(_HERE, "..", "Linux"))
SDK_ROOT = os.environ.get("UNITREE_MOTOR_SDK", _DEFAULT_SDK_ROOT)
MOTOR_CTRL = os.path.join(SDK_ROOT, "build", "motor_ctrl")
CONFIG_DIR = os.path.normpath(os.path.join(_HERE, "..", "config"))
STAND_CONFIG = os.path.join(CONFIG_DIR, "stand_config.json")

GEAR_RATIO = 6.33
SUDO_PASSWORD = "1"
NEED_SUDO = os.geteuid() != 0

LEG_NAMES = {"fl": "左前腿", "fr": "右前腿", "rl": "左后腿", "rr": "右后腿"}


def _sudo(cmd):
    return (["sudo", "-S", "-p", ""] + cmd) if NEED_SUDO else cmd


def read_rotor(port, mid):
    """读单个电机当前转子角(rad)；读不到返回 None。"""
    proc = subprocess.run(
        _sudo([MOTOR_CTRL, port, str(mid), "read"]),
        input=(SUDO_PASSWORD + "\n") if NEED_SUDO else None,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=8,
    )
    for line in (proc.stdout or "").splitlines():
        m = re.match(rf"\s*ANGLE\s+id={mid}\s+ok=([01])(.*)", line)
        if m:
            if m.group(1) != "1":
                return None
            mm = re.search(r"rotor=(-?\d+(?:\.\d+)?)", m.group(2))
            return float(mm.group(1)) if mm else None
    return None


def main():
    leg = (sys.argv[1] if len(sys.argv) > 1 else "fl").lower()
    if leg not in LEG_NAMES:
        sys.exit(f"未知腿名: {leg}（可选 fl/fr/rl/rr）")
    if not os.path.isfile(MOTOR_CTRL):
        sys.exit(f"未找到 {MOTOR_CTRL}，请先在 build/ 执行 cmake .. && make motor_ctrl")
    if not os.path.isfile(STAND_CONFIG):
        sys.exit(f"未找到 {STAND_CONFIG}，请先完成标定")

    cfg = json.load(open(STAND_CONFIG, encoding="utf-8"))
    ex = cfg.get("execute", {})
    thr = float(ex.get("verify_threshold_rad", 0.20))
    vmax_joint = float(ex.get("max_joint_vel_rad_s", 0.05))
    min_dur = float(ex.get("min_duration_s", 2.0))
    rate = float(ex.get("rate_hz", 100))
    kp = float(ex.get("k_p", 2.0))
    kw = float(ex.get("k_w", 1.0))

    # 选出该腿的 3 个关节（名字以 leg+"_" 开头），并确认同一路串口
    joints = [j for j in cfg["joints"] if str(j.get("name", "")).startswith(leg + "_")]
    if not joints:
        sys.exit(f"配置里没有 {leg} 开头的关节")
    ports = {j["port"] for j in joints}
    if len(ports) != 1:
        sys.exit(f"{leg} 的关节不在同一路串口上: {ports}，本脚本只支持单总线单腿")
    port = ports.pop()

    miss = [j["name"] for j in joints
            if j.get("prone_rotor") is None or j.get("delta_rotor") is None]
    if miss:
        sys.exit(f"以下关节缺 prone_rotor/delta_rotor，无法验证: {', '.join(miss)}")

    print(f"\n=== 单腿验证: {leg} ({LEG_NAMES[leg]})  串口 {port} ===")
    print(f"增益 K_P={kp} K_W={kw}  限速 {vmax_joint} rad/s(关节)  阈值 {thr} rad(转子)")
    print("正在读取当前角度做安全校验 ...")

    cur = {}
    bad = []
    for j in joints:
        r = read_rotor(port, j["id"])
        if r is None:
            bad.append(f"{j['name']}(无响应)")
            continue
        cur[j["id"]] = r
        dev = abs(r - j["prone_rotor"])
        if dev > thr:
            bad.append(f"{j['name']}(当前偏离趴姿 {dev:.3f} > {thr})")
    if bad:
        print("\n[拒绝] 安全校验未通过，未发送任何指令：")
        for b in bad:
            print("   - " + b)
        print("请确认：① 该腿电机在电机模式且接线正常；② 机器人已摆回标定时的趴姿。")
        sys.exit(2)

    # 计划：目标 = 当前 + delta；打印每个关节方向与幅度
    targets = {j["id"]: cur[j["id"]] + j["delta_rotor"] for j in joints}
    print("\n本次将执行的动作（趴 -> 站）：")
    print(f"  {'关节':10} {'当前°':>9} {'目标°':>9} {'Δ关节°':>9}  方向")
    max_move = 0.0
    for j in joints:
        i = j["id"]
        cur_deg = cur[i] / GEAR_RATIO * 180 / 3.141592653589793
        tgt_deg = targets[i] / GEAR_RATIO * 180 / 3.141592653589793
        d_deg = j["delta_rotor"] / GEAR_RATIO * 180 / 3.141592653589793
        arrow = "＋(增大)" if j["delta_rotor"] > 0 else ("－(减小)" if j["delta_rotor"] < 0 else "·(不动)")
        print(f"  {j['name']:10} {cur_deg:9.2f} {tgt_deg:9.2f} {d_deg:9.2f}  {arrow}")
        max_move = max(max_move, abs(targets[i] - cur[i]))

    vmax_rotor = vmax_joint * GEAR_RATIO
    duration = max(min_dur, max_move / vmax_rotor if vmax_rotor > 0 else min_dur)
    N = max(1, int(duration * rate))
    print(f"\n预计耗时 {duration:.1f}s（{N} 步）。请确认机器人已架空、{LEG_NAMES[leg]}周围已清空。")
    try:
        ans = input("确认开始？输入 y 回车继续，其它任意键取消: ").strip().lower()
    except (EOFError, KeyboardInterrupt):
        ans = ""
    if ans != "y":
        sys.exit("已取消，未发送任何指令。")

    # 在该腿单路总线上开 servo 进程
    servo = subprocess.Popen(
        _sudo([MOTOR_CTRL, port, "servo", f"{kp}", f"{kw}"]),
        stdin=subprocess.PIPE, stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT, text=True, bufsize=1,
    )
    if NEED_SUDO:
        try:
            servo.stdin.write(SUDO_PASSWORD + "\n"); servo.stdin.flush()
        except (BrokenPipeError, OSError):
            pass
    time.sleep(0.3)  # 等 servo 起来（含 sudo 鉴权）

    def send(line):
        try:
            servo.stdin.write(line + "\n"); servo.stdin.flush()
        except (BrokenPipeError, OSError):
            print("[警告] servo 管道断开")

    def release():
        send("stop")
        time.sleep(0.5)
        if servo.poll() is None:
            try:
                servo.terminate(); servo.wait(timeout=2)
            except Exception:
                try:
                    servo.kill()
                except Exception:
                    pass

    try:
        ids = [j["id"] for j in joints]
        dt = 1.0 / rate
        print("\n开始缓慢移动 ... （随时 Ctrl-C 立即释放）")
        for k in range(1, N + 1):
            a = k / N
            parts = []
            for i in ids:
                pos = cur[i] + a * (targets[i] - cur[i])
                parts.append(f"{i} {pos:.5f}")
            send(" ".join(parts))
            time.sleep(dt)
        print("\n[完成] 已到达站姿目标，servo 正保持位置。")
        print("请检查这条腿：是否朝『站起来』方向收拢？哪个关节方向不对？")
        try:
            input("观察完毕后，按回车释放电机 ...")
        except (EOFError, KeyboardInterrupt):
            pass
    except KeyboardInterrupt:
        print("\n[中止] 收到 Ctrl-C，立即释放 ...")
    finally:
        release()
        print("[退出] 已发送停止脉冲并退出。")


if __name__ == "__main__":
    main()
