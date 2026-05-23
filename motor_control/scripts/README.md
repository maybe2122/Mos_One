# motor_id_gui.py — GO-M8010-6 电机 ID 管理与驱动测试 GUI

Tkinter 图形界面，封装宇树 GO-M8010-6 电机随附工具（`swboot` / `changeid` /
`swmotor`）和一个基于官方 SDK 的可传参驱动小工具 `motor_ctrl`，方便在 4 路 USB‑RS485
模块上批量查看 / 修改电机 ID，以及做单电机驱动 / 停止测试。

## 功能一览

| 区块       | 操作                       | 对应底层命令                                      |
|------------|----------------------------|---------------------------------------------------|
| 串口       | 选择 `/dev/ttyUSB*` 并刷新 | —                                                 |
| 操作       | 查看电机 ID                | `swboot <port>`                                   |
|            | 扫描全部 `ttyUSB*`         | 依次对每路跑 `swboot`，输出汇总                   |
|            | 修改电机 ID                | `changeid <port> <old> <new>`                     |
|            | 切回电机模式               | `swmotor <port>`                                  |
|            | 切回全部 `ttyUSB*`         | 依次对每路跑 `swmotor`                            |
| 电机控制   | 驱动转动 / 停止            | `motor_ctrl <port> <id> drive|stop ...`           |
| 状态栏     | 取消当前命令               | 向当前子进程发 SIGTERM / SIGKILL                  |

- 扫描会自动把检测到的 ID 填进「原 ID」「驱动电机 ID」下拉框。
- 「切回电机模式」遇到无固件电机可能久等，可点「取消当前命令」中止。
- 「停止」按钮先杀掉驱动子进程，再补一段 500 ms 的 `mode=0` 停止脉冲。

## 依赖

- Linux + Python 3（标准库 `tkinter` 必装：`sudo apt install python3-tk`）
- 宇树 GO-M8010-6 SDK 解压目录，包含：
  - `motor_tools/Unitree_MotorTools_v0.2.0_x86_64_Linux/{swboot,changeid,swmotor}`
  - `build/motor_ctrl`（需要先在 SDK 目录下编译，见下）
- USB‑RS485 模块，电机已正确接线、上电
- `sudo`（脚本已内置免密策略，详见「权限与密码」一节）

## 编译 `motor_ctrl`

`motor_ctrl` 是仿照 SDK 示例 `example/main.cpp` / `stop.cpp` 写的可传参版本，
源码与 `CMakeLists.txt` 修改都已合入 SDK 目录。首次使用前在 SDK 根目录执行：

```bash
cd <SDK_ROOT>
mkdir -p build && cd build
cmake ..
make motor_ctrl
```

产物 `build/motor_ctrl` 必须存在，否则 GUI 启动时会弹警告，「驱动转动」按钮不可用。

参数：

```
motor_ctrl <port> <id> drive [speed_rad_s] [duration_ms]
motor_ctrl <port> <id> stop  [duration_ms]
```

- `duration_ms = 0`：永远循环，直到被外部 SIGTERM
- 收到 SIGTERM/SIGINT 时，若处于 drive 模式会自动补发 200 ms 的停止脉冲

## 路径配置

脚本通过 `SDK_ROOT` 常量定位 SDK；优先读环境变量 `UNITREE_MOTOR_SDK`，
未设置时回退到一个默认硬编码路径（见脚本顶部）。

如果 SDK 位置变动，建议：

```bash
export UNITREE_MOTOR_SDK="/your/path/to/Linux平台电机使用例程(包含SDK)"
python3 motor_id_gui.py
```

涉及到的目录：

| 变量         | 路径                                                       |
|--------------|------------------------------------------------------------|
| `SDK_ROOT`   | SDK 根目录                                                 |
| `TOOL_DIR`   | `$SDK_ROOT/motor_tools/Unitree_MotorTools_v0.2.0_x86_64_Linux` |
| `BUILD_DIR`  | `$SDK_ROOT/build`                                          |
| `MOTOR_CTRL` | `$BUILD_DIR/motor_ctrl`                                    |

## 运行

```bash
python3 /home/maybe/code/rl/Stackforce-simready-mos2026/motor_control/scripts/motor_id_gui.py
```

或加入 `PATH` / 桌面快捷方式自取。脚本权限为 `700`，仅本用户可读 / 执行。

## 权限与密码

`swboot` / `changeid` / `swmotor` / `motor_ctrl` 都需要 root 才能访问串口。
脚本顶部 `SUDO_PASSWORD = "1"` 用于通过 `sudo -S` 自动喂密码，免去每次输入。

> ⚠️ 这是明文密码。脚本权限已限制为 `700`（`chmod 700 motor_id_gui.py`），
> 仅本用户可读。**不要把该文件提交到 git 或分享给他人**；如需分享，
> 先把 `SUDO_PASSWORD` 那一行删掉。
>
> 如果系统密码不是 `1`，请修改脚本顶部的 `SUDO_PASSWORD`。

更安全的替代方案（脚本里没启用，建议生产环境改用）：

```bash
sudo tee /etc/sudoers.d/unitree-motor-tools <<'EOF'
maybe ALL=(root) NOPASSWD: /media/maybe/.../motor_tools/Unitree_MotorTools_v0.2.0_x86_64_Linux/swboot, \
                           /media/maybe/.../motor_tools/Unitree_MotorTools_v0.2.0_x86_64_Linux/changeid, \
                           /media/maybe/.../motor_tools/Unitree_MotorTools_v0.2.0_x86_64_Linux/swmotor, \
                           /media/maybe/.../build/motor_ctrl
EOF
```

配好后把脚本里的 `SUDO_PASSWORD` 改成空字符串即可。

## 典型工作流

1. **接好硬件**：USB‑RS485 模块连主机，4 路总线接对应电机并上电。
2. 启动 GUI，确认下方串口下拉框列出了 `/dev/ttyUSB0..3`。
3. 点 **扫描全部 ttyUSB***，看汇总：哪一路总线上有哪些 ID。
4. 如果要改 ID：
   - 上方串口下拉框选中对应路。
   - 「原 ID」「新 ID」填好，点「修改电机 ID」。
5. 改完之后点 **切回全部 ttyUSB***，把所有电机切回电机模式
   （工厂模式下电机不响应运动指令）。
6. 驱动测试：
   - 上方串口下拉框选目标路。
   - 「电机控制」区填电机 ID 和转速，点「▶ 驱动转动」。
   - 测完点「■ 停止」。

## 故障排查

| 现象 | 处理 |
|------|------|
| 启动弹「缺少工具」 | SDK 路径不对，`UNITREE_MOTOR_SDK` 设错或 SDK 没挂载（移动硬盘场景） |
| 启动弹「缺少 motor_ctrl」 | 没编译，回 SDK 目录 `cd build && cmake .. && make motor_ctrl` |
| `swmotor` 多电机时长时间无输出 | 工具会逐一尝试所有 ID，遇到无固件电机较慢；点「取消当前命令」即可 |
| 「驱动转动」点击无反应 | 电机仍在工厂模式（绿灯快闪 3 次/秒），先点「切回电机模式」 |
| 总是要求输密码 | `SUDO_PASSWORD` 跟系统密码不一致，或 sudo 配置了 `requiretty` |
| 取消按钮按了进程不死 | 子进程在 root 下运行，sudo 转发 SIGTERM 失败；可在终端 `sudo pkill -f motor_ctrl` 兜底 |

## 文件清单

```
scripts/
├── motor_id_gui.py     # 本 GUI 主程序（含明文 sudo 密码，权限 700）
└── README.md           # 本文档
```

SDK 目录侧的新增 / 修改：

```
<SDK_ROOT>/
├── example/motor_ctrl.cpp   # 新增：可传参的驱动 / 停止工具源码
├── CMakeLists.txt           # 新增 motor_ctrl target
└── build/motor_ctrl         # 编译产物
```
