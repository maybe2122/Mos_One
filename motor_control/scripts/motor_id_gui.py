#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GO-M8010-6 电机 ID 管理界面
功能：
  1. 查看电机 ID（swboot 进入工厂模式）
  2. 修改电机 ID（changeid）
  3. 切回电机模式（swmotor）
"""

import os
import re
import subprocess
import threading
import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext

# SDK 所在目录（GO-M8010-6 SDK 解压目录）。
# 可用环境变量 UNITREE_MOTOR_SDK 覆盖。
SDK_ROOT = os.environ.get(
    "UNITREE_MOTOR_SDK",
    "/media/maybe/新加卷/Diy/GO-M8010-6电机使用教程/Linux平台教程/"
    "Linux平台电机使用例程(包含SDK)",
)
TOOL_DIR = os.path.join(
    SDK_ROOT, "motor_tools", "Unitree_MotorTools_v0.2.0_x86_64_Linux"
)
BUILD_DIR = os.path.join(SDK_ROOT, "build")
MOTOR_CTRL = os.path.join(BUILD_DIR, "motor_ctrl")

SUDO_PASSWORD = "1"
DEFAULT_SPEED = 6.28 * 6.33  # 与 example/main.cpp 一致


def list_serial_ports():
    ports = []
    for name in os.listdir("/dev"):
        if name.startswith(("ttyUSB", "ttyACM", "ttyS")):
            ports.append("/dev/" + name)
    ports.sort()
    if "/dev/ttyUSB0" not in ports:
        ports.insert(0, "/dev/ttyUSB0")
    return ports


class MotorIdApp:
    def __init__(self, root):
        self.root = root
        root.title("GO-M8010-6 电机 ID 管理")
        root.geometry("760x560")

        self.running = False
        self.need_sudo = os.geteuid() != 0
        self.drive_proc = None  # 持续运行的驱动子进程
        self.cmd_proc = None    # 当前前台命令子进程（swboot/changeid/swmotor）
        self.cancel_requested = threading.Event()  # 多步骤命令的取消标志

        self._build_ui()
        self._check_tools()

        root.protocol("WM_DELETE_WINDOW", self._on_close)

    # ------------------------------------------------------------------ UI
    def _build_ui(self):
        pad = {"padx": 6, "pady": 4}

        # 串口选择
        top = ttk.LabelFrame(self.root, text="串口")
        top.pack(fill="x", **pad)

        ttk.Label(top, text="串口号:").grid(row=0, column=0, sticky="w", **pad)
        self.port_var = tk.StringVar(value="/dev/ttyUSB0")
        self.port_combo = ttk.Combobox(
            top, textvariable=self.port_var, values=list_serial_ports(), width=22
        )
        self.port_combo.grid(row=0, column=1, sticky="w", **pad)

        ttk.Button(top, text="刷新", command=self._refresh_ports).grid(
            row=0, column=2, **pad
        )

        # 操作区
        ops = ttk.LabelFrame(self.root, text="操作")
        ops.pack(fill="x", **pad)

        # 查看电机 ID
        self.btn_view = ttk.Button(
            ops, text="1. 查看电机 ID  (swboot)", command=self.on_view_id, width=28
        )
        self.btn_view.grid(row=0, column=0, columnspan=2, sticky="w", **pad)

        self.btn_view_all = ttk.Button(
            ops,
            text="扫描全部 ttyUSB*",
            command=self.on_view_id_all,
            width=18,
        )
        self.btn_view_all.grid(row=0, column=2, columnspan=2, sticky="w", **pad)

        # 修改电机 ID
        ttk.Label(ops, text="原 ID:").grid(row=1, column=0, sticky="e", **pad)
        self.old_id_var = tk.StringVar(value="0")
        self.old_id_combo = ttk.Combobox(
            ops,
            textvariable=self.old_id_var,
            values=[str(i) for i in range(16)],
            width=6,
        )
        self.old_id_combo.grid(row=1, column=1, sticky="w", **pad)

        ttk.Label(ops, text="新 ID:").grid(row=1, column=2, sticky="e", **pad)
        self.new_id_var = tk.StringVar(value="1")
        ttk.Spinbox(
            ops, from_=0, to=15, width=6, textvariable=self.new_id_var
        ).grid(row=1, column=3, sticky="w", **pad)

        self.btn_change = ttk.Button(
            ops, text="2. 修改电机 ID  (changeid)", command=self.on_change_id, width=28
        )
        self.btn_change.grid(row=2, column=0, columnspan=4, sticky="w", **pad)

        # 切回电机模式
        self.btn_motor = ttk.Button(
            ops,
            text="3. 切回电机模式  (swmotor)",
            command=self.on_switch_motor,
            width=28,
        )
        self.btn_motor.grid(row=3, column=0, columnspan=2, sticky="w", **pad)

        self.btn_motor_all = ttk.Button(
            ops,
            text="切回全部 ttyUSB*",
            command=self.on_switch_motor_all,
            width=18,
        )
        self.btn_motor_all.grid(row=3, column=2, columnspan=2, sticky="w", **pad)

        # ---- 电机控制（驱动 / 停止）----
        ctrl = ttk.LabelFrame(self.root, text="电机控制（需要电机已处于电机模式）")
        ctrl.pack(fill="x", **pad)

        ttk.Label(ctrl, text="电机 ID:").grid(row=0, column=0, sticky="e", **pad)
        self.drive_id_var = tk.StringVar(value="")
        self.drive_id_combo = ttk.Combobox(
            ctrl,
            textvariable=self.drive_id_var,
            values=[str(i) for i in range(16)],
            width=6,
        )
        self.drive_id_combo.grid(row=0, column=1, sticky="w", **pad)

        ttk.Label(ctrl, text="转速 (rad/s):").grid(row=0, column=2, sticky="e", **pad)
        self.speed_var = tk.StringVar(value=f"{DEFAULT_SPEED:.3f}")
        ttk.Entry(ctrl, textvariable=self.speed_var, width=10).grid(
            row=0, column=3, sticky="w", **pad
        )

        self.btn_drive = ttk.Button(
            ctrl, text="▶ 驱动转动", command=self.on_drive, width=14
        )
        self.btn_drive.grid(row=1, column=0, columnspan=2, sticky="w", **pad)

        self.btn_stop = ttk.Button(
            ctrl, text="■ 停止", command=self.on_motor_stop, width=14, state="disabled"
        )
        self.btn_stop.grid(row=1, column=2, columnspan=2, sticky="w", **pad)

        # 提示
        tip = (
            "提示：查看 / 修改 ID 会让电机进入工厂模式（背部绿灯每秒快闪 3 次）。\n"
            "操作前请确保所有电机已停止，主机不再发送运动控制指令。\n"
            "完成后必须点击「切回电机模式」，否则重新上电仍在工厂模式。"
        )
        ttk.Label(self.root, text=tip, foreground="#a60", justify="left").pack(
            fill="x", **pad
        )

        # 输出
        out = ttk.LabelFrame(self.root, text="输出")
        out.pack(fill="both", expand=True, **pad)
        self.output = scrolledtext.ScrolledText(out, height=14, font=("Monospace", 10))
        self.output.pack(fill="both", expand=True)

        # 状态栏 + 取消按钮
        bar = ttk.Frame(self.root)
        bar.pack(fill="x")
        self.status_var = tk.StringVar(value="就绪")
        ttk.Label(
            bar, textvariable=self.status_var, anchor="w", relief="sunken"
        ).pack(side="left", fill="x", expand=True)
        self.btn_cancel = ttk.Button(
            bar, text="取消当前命令", command=self.on_cancel_cmd, state="disabled"
        )
        self.btn_cancel.pack(side="right", padx=4, pady=2)

    def _refresh_ports(self):
        self.port_combo["values"] = list_serial_ports()

    def _check_tools(self):
        missing = [
            t
            for t in ("swboot", "changeid", "swmotor")
            if not os.path.isfile(os.path.join(TOOL_DIR, t))
        ]
        if missing:
            messagebox.showerror(
                "缺少工具", f"在 {TOOL_DIR} 下未找到: {', '.join(missing)}"
            )
        if not os.path.isfile(MOTOR_CTRL):
            messagebox.showwarning(
                "缺少 motor_ctrl",
                f"未找到 {MOTOR_CTRL}\n请在 build/ 目录执行: cmake .. && make motor_ctrl",
            )

    # ------------------------------------------------------------ Actions
    def on_view_id(self):
        self._run([os.path.join(TOOL_DIR, "swboot"), self.port_var.get()],
                  title="查看电机 ID",
                  post_parse=True)

    def on_change_id(self):
        old = self.old_id_var.get().strip()
        new = self.new_id_var.get().strip()
        if not (old.isdigit() and new.isdigit()):
            messagebox.showwarning("输入错误", "原 ID / 新 ID 必须是数字")
            return
        if old == new:
            messagebox.showwarning("输入错误", "原 ID 与新 ID 不能相同")
            return
        if not messagebox.askyesno(
            "确认",
            f"将串口 {self.port_var.get()} 上所有 ID={old} 的电机改为 ID={new}？\n\n"
            "请确认电机已停止、主机已不再发送控制指令。",
        ):
            return
        self._run(
            [os.path.join(TOOL_DIR, "changeid"), self.port_var.get(), old, new],
            title=f"修改电机 ID: {old} -> {new}",
        )

    def on_switch_motor(self):
        self._run([os.path.join(TOOL_DIR, "swmotor"), self.port_var.get()],
                  title="切回电机模式")

    # -- 多串口批量操作 -------------------------------------------------
    def _list_ttyusb_ports(self):
        ports = sorted(
            "/dev/" + n for n in os.listdir("/dev") if n.startswith("ttyUSB")
        )
        return ports

    def on_view_id_all(self):
        ports = self._list_ttyusb_ports()
        if not ports:
            messagebox.showwarning("无串口", "未找到任何 /dev/ttyUSB*")
            return
        self._run_multi_port(
            tool=os.path.join(TOOL_DIR, "swboot"),
            extra_args=[],
            ports=ports,
            title="扫描全部串口 (swboot)",
            post_parse=True,
        )

    def on_switch_motor_all(self):
        ports = self._list_ttyusb_ports()
        if not ports:
            messagebox.showwarning("无串口", "未找到任何 /dev/ttyUSB*")
            return
        self._run_multi_port(
            tool=os.path.join(TOOL_DIR, "swmotor"),
            extra_args=[],
            ports=ports,
            title="切回全部串口 (swmotor)",
            post_parse=False,
        )

    def _run_multi_port(self, tool, extra_args, ports, title, post_parse):
        if self.running:
            messagebox.showinfo("忙", "请等待当前命令完成，或点击「取消当前命令」")
            return

        self.cancel_requested.clear()
        self.running = True
        for b in (
            self.btn_view, self.btn_change, self.btn_motor, self.btn_drive,
            self.btn_view_all, self.btn_motor_all,
        ):
            b.config(state="disabled")
        self.btn_cancel.config(state="normal")

        self._log("\n" + "=" * 60)
        self._log(f"[{title}] 串口列表: {', '.join(ports)}")
        self.status_var.set(f"{title} 进行中（可取消）")

        threading.Thread(
            target=self._multi_port_worker,
            args=(tool, extra_args, ports, title, post_parse),
            daemon=True,
        ).start()

    def _multi_port_worker(self, tool, extra_args, ports, title, post_parse):
        per_port_ids = {}
        try:
            for port in ports:
                if self.cancel_requested.is_set():
                    self.root.after(0, self._log, "[已取消] 跳过剩余串口")
                    break

                cmd = [tool, port] + list(extra_args)
                if self.need_sudo:
                    full_cmd = ["sudo", "-S", "-p", ""] + cmd
                else:
                    full_cmd = cmd

                shown = ["sudo"] + cmd if self.need_sudo else cmd
                self.root.after(0, self._log, "\n--- " + port + " ---")
                self.root.after(0, self._log, "$ " + " ".join(shown))

                try:
                    proc = subprocess.Popen(
                        full_cmd,
                        stdin=subprocess.PIPE,
                        stdout=subprocess.PIPE,
                        stderr=subprocess.STDOUT,
                        text=True,
                        bufsize=1,
                    )
                except FileNotFoundError as e:
                    self.root.after(0, self._log, f"[错误] {e}")
                    continue

                if self.need_sudo:
                    try:
                        proc.stdin.write(SUDO_PASSWORD + "\n")
                        proc.stdin.flush()
                    except (BrokenPipeError, OSError):
                        pass
                try:
                    proc.stdin.close()
                except (BrokenPipeError, OSError):
                    pass

                self.cmd_proc = proc
                buf = []
                for line in proc.stdout:
                    buf.append(line)
                    self.root.after(0, self._log, line.rstrip("\n"))
                proc.wait()
                self.cmd_proc = None
                self.root.after(0, self._log, f"[{port} 退出码 {proc.returncode}]")

                if post_parse:
                    ids = self._parse_ids("".join(buf))
                    per_port_ids[port] = ids

            # 汇总
            if post_parse:
                self.root.after(0, self._log, "\n===== 扫描汇总 =====")
                all_ids = []
                for p in ports:
                    ids = per_port_ids.get(p, [])
                    if ids:
                        self.root.after(
                            0, self._log,
                            f"{p}: ID = {', '.join(map(str, ids))}",
                        )
                        for i in ids:
                            if i not in all_ids:
                                all_ids.append(i)
                    else:
                        self.root.after(0, self._log, f"{p}: (未检测到 / 已取消)")
                if all_ids:
                    self.root.after(0, self._update_detected_ids, all_ids)
        finally:
            self.root.after(0, self._done, title)

    # -- 电机驱动 / 停止 ----------------------------------------------
    def _validate_drive_id(self):
        s = self.drive_id_var.get().strip()
        if not s:
            messagebox.showwarning("缺少电机 ID", "请先填写电机 ID（0~15）后再驱动。")
            return None
        if not s.isdigit():
            messagebox.showwarning("输入错误", "电机 ID 必须是 0~15 的数字。")
            return None
        n = int(s)
        if not 0 <= n <= 15:
            messagebox.showwarning("输入错误", "电机 ID 必须在 0~15 范围内。")
            return None
        return str(n)

    def on_drive(self):
        if self.drive_proc is not None:
            messagebox.showinfo("提示", "电机已在驱动中，请先停止。")
            return
        if self.running:
            messagebox.showinfo("忙", "请等待当前命令完成")
            return
        if not os.path.isfile(MOTOR_CTRL):
            messagebox.showerror("缺少 motor_ctrl", f"未找到 {MOTOR_CTRL}")
            return

        mid = self._validate_drive_id()
        if mid is None:
            return

        speed_s = self.speed_var.get().strip()
        try:
            speed = float(speed_s)
        except ValueError:
            messagebox.showwarning("输入错误", "转速必须是数字（rad/s）。")
            return

        cmd = [MOTOR_CTRL, self.port_var.get(), mid, "drive", f"{speed}", "0"]
        if self.need_sudo:
            full_cmd = ["sudo", "-S", "-p", ""] + cmd
        else:
            full_cmd = cmd

        self._log("\n" + "=" * 60)
        self._log(f"[驱动 id={mid} speed={speed}]  $ {' '.join(['sudo'] + cmd if self.need_sudo else cmd)}")
        self.status_var.set(f"驱动中: id={mid} speed={speed}")

        # 锁定其他动作按钮，仅保留「停止」可用
        for b in (self.btn_view, self.btn_change, self.btn_motor, self.btn_drive):
            b.config(state="disabled")
        self.btn_stop.config(state="normal")

        try:
            proc = subprocess.Popen(
                full_cmd,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
            )
        except FileNotFoundError as e:
            self._log(f"[错误] {e}")
            self._drive_finished()
            return

        if self.need_sudo:
            try:
                proc.stdin.write(SUDO_PASSWORD + "\n")
                proc.stdin.flush()
            except (BrokenPipeError, OSError):
                pass
        try:
            proc.stdin.close()
        except (BrokenPipeError, OSError):
            pass

        self.drive_proc = proc
        threading.Thread(target=self._drive_reader, args=(proc,), daemon=True).start()

    def _drive_reader(self, proc):
        try:
            for line in proc.stdout:
                self.root.after(0, self._log, line.rstrip("\n"))
            proc.wait()
            self.root.after(0, self._log, f"[驱动进程退出码 {proc.returncode}]")
        finally:
            self.root.after(0, self._drive_finished)

    def _drive_finished(self):
        self.drive_proc = None
        self.btn_stop.config(state="disabled")
        for b in (self.btn_view, self.btn_change, self.btn_motor, self.btn_drive):
            b.config(state="normal")
        self.status_var.set("驱动已停止")

    def on_motor_stop(self):
        mid = self._validate_drive_id()
        if mid is None:
            return

        proc = self.drive_proc
        if proc is not None and proc.poll() is None:
            self._log("[停止] 终止驱动子进程 ...")
            try:
                proc.terminate()
                try:
                    proc.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    proc.kill()
            except Exception as e:
                self._log(f"[警告] 终止驱动失败: {e}")

        # 再发一段 mode=0 的停止脉冲，确保电机真正停下
        cmd = [MOTOR_CTRL, self.port_var.get(), mid, "stop", "500"]
        if self.need_sudo:
            full_cmd = ["sudo", "-S", "-p", ""] + cmd
        else:
            full_cmd = cmd
        self._log(f"[停止脉冲]  $ {' '.join(['sudo'] + cmd if self.need_sudo else cmd)}")

        def _send_stop():
            try:
                p = subprocess.Popen(
                    full_cmd,
                    stdin=subprocess.PIPE,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                    bufsize=1,
                )
                if self.need_sudo:
                    try:
                        p.stdin.write(SUDO_PASSWORD + "\n")
                        p.stdin.flush()
                    except (BrokenPipeError, OSError):
                        pass
                try:
                    p.stdin.close()
                except (BrokenPipeError, OSError):
                    pass
                for line in p.stdout:
                    self.root.after(0, self._log, line.rstrip("\n"))
                p.wait()
                self.root.after(0, self._log, f"[停止脉冲退出码 {p.returncode}]")
            finally:
                self.root.after(0, self._drive_finished)

        threading.Thread(target=_send_stop, daemon=True).start()

    def _on_close(self):
        if self.drive_proc is not None and self.drive_proc.poll() is None:
            try:
                self.drive_proc.terminate()
                try:
                    self.drive_proc.wait(timeout=1)
                except subprocess.TimeoutExpired:
                    self.drive_proc.kill()
            except Exception:
                pass
        self.root.destroy()

    # ------------------------------------------------------------ Runner
    def _run(self, cmd, title="", post_parse=False):
        if self.running:
            messagebox.showinfo("忙", "请等待当前命令完成，或点击「取消当前命令」")
            return

        if self.need_sudo:
            full_cmd = ["sudo", "-S", "-p", ""] + cmd
        else:
            full_cmd = cmd

        self._log("\n" + "=" * 60)
        shown = ["sudo"] + cmd if self.need_sudo else cmd
        self._log(f"[{title}]  $ {' '.join(shown)}")
        self.status_var.set(f"运行中: {title}（可点「取消当前命令」中止）")

        try:
            proc = subprocess.Popen(
                full_cmd,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
            )
        except FileNotFoundError as e:
            self._log(f"[错误] {e}")
            return

        if self.need_sudo:
            try:
                proc.stdin.write(SUDO_PASSWORD + "\n")
                proc.stdin.flush()
            except (BrokenPipeError, OSError):
                pass
        try:
            proc.stdin.close()
        except (BrokenPipeError, OSError):
            pass

        self.running = True
        self.cmd_proc = proc
        self.cancel_requested.clear()
        for b in (
            self.btn_view, self.btn_change, self.btn_motor, self.btn_drive,
            self.btn_view_all, self.btn_motor_all,
        ):
            b.config(state="disabled")
        self.btn_cancel.config(state="normal")

        t = threading.Thread(
            target=self._worker,
            args=(proc, title, post_parse),
            daemon=True,
        )
        t.start()

    def _worker(self, proc, title, post_parse):
        output_buf = []
        try:
            for line in proc.stdout:
                output_buf.append(line)
                self.root.after(0, self._log, line.rstrip("\n"))
            proc.wait()
            rc = proc.returncode
            self.root.after(0, self._log, f"[退出码 {rc}]")
            if post_parse:
                ids = self._parse_ids("".join(output_buf))
                if ids:
                    self.root.after(
                        0, self._log, f"==> 检测到电机 ID: {', '.join(map(str, ids))}"
                    )
                    self.root.after(0, self._update_detected_ids, ids)
                else:
                    self.root.after(
                        0,
                        self._log,
                        "==> 未解析到电机 ID（请查看上方原始输出）",
                    )
        except FileNotFoundError as e:
            self.root.after(0, self._log, f"[错误] {e}")
        finally:
            self.root.after(0, self._done, title)

    def _parse_ids(self, text):
        # 兼容多种 swboot 输出格式：
        #   "ID: 0"、"ID=0"、"ID 0"
        #   "motor[0]"、"motor 0"、"motor_0"
        #   "id0"
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

    def _update_detected_ids(self, ids):
        """把扫到的电机 ID 填到「原 ID」与「驱动电机 ID」下拉框。"""
        values = [str(i) for i in ids]
        rest = [str(i) for i in range(16) if i not in ids]
        self.old_id_combo["values"] = values + rest
        self.old_id_var.set(values[0])
        self.drive_id_combo["values"] = values + rest
        if not self.drive_id_var.get():
            self.drive_id_var.set(values[0])
        self.status_var.set(f"检测到电机 ID: {', '.join(values)}")

    def _done(self, title):
        self.running = False
        self.cmd_proc = None
        for b in (
            self.btn_view, self.btn_change, self.btn_motor, self.btn_drive,
            self.btn_view_all, self.btn_motor_all,
        ):
            b.config(state="normal")
        self.btn_cancel.config(state="disabled")
        self.status_var.set(f"完成: {title}")

    def on_cancel_cmd(self):
        self.cancel_requested.set()  # 让多步骤循环停止处理下一个串口
        proc = self.cmd_proc
        if proc is None or proc.poll() is not None:
            self.status_var.set("已请求取消，等待循环退出 ...")
            return
        self._log("[取消] 正在终止当前命令 ...")
        self.status_var.set("正在取消 ...")
        # sudo -S 启动的子进程也需要一并终止：使用 SIGTERM；
        # 必要时升级为 SIGKILL。
        def _killer(p):
            try:
                p.terminate()
                try:
                    p.wait(timeout=2)
                    self.root.after(0, self._log, "[取消] 已终止")
                except subprocess.TimeoutExpired:
                    p.kill()
                    self.root.after(0, self._log, "[取消] 已强制 kill")
            except Exception as e:
                self.root.after(0, self._log, f"[取消] 失败: {e}")
        threading.Thread(target=_killer, args=(proc,), daemon=True).start()

    def _log(self, msg):
        self.output.insert("end", msg + "\n")
        self.output.see("end")


def main():
    root = tk.Tk()
    MotorIdApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
