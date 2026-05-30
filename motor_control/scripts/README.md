# GO-M8010-6 电机 ID 管理与驱动测试

封装宇树 GO-M8010-6 电机随附工具（`swboot` / `changeid` / `swmotor`）和一个基于官方
SDK 的可传参驱动 / 读取小工具 `motor_ctrl`，方便在 4 路 USB‑RS485 模块上批量查看 /
修改电机 ID、做单电机驱动 / 停止测试，以及读取所有电机当前角度。

前端为网页版 `motor_web.py`：纯标准库无依赖、浏览器渲染中文（不会出现字体方块）、
默认只监听本机：

```bash
python3 scripts/motor_web.py   # 浏览器开 http://127.0.0.1:8000
```

> 默认仅监听 `127.0.0.1`。如需在另一台机器的浏览器访问：
> `HOST=0.0.0.0 PORT=8000 python3 scripts/motor_web.py`（注意这会把电机控制暴露到局域网）。

### robot_web.py —— 四足机器人操控（站立）

仿手柄的整机操控面板，本期实现「站立」动作：① 趴姿标定 → ② 站姿标定（手扶撑起）→
③ 计算各关节趴→站转动量并存 `config/stand_config.json` → 站立时先安全校验当前角是否贴近
配置趴姿，通过后按关节限速（默认 ≤0.1 rad/s）缓慢起身。底层复用 `motor_ctrl`
（`read` 读角；新增 `servo` 流式位置伺服，插值在 Python 端算）。设计细节见
`motor_control/doc/四足站立控制设计.md`。

**单腿验证 / 方向验证**（首次标定后、整机站立前务必先逐腿核对）：面板「单腿验证」
下拉选腿（FL/FR/RL/RR），只在该腿对应的那一路总线上发力、其它腿完全不碰，走与整机
站立同一套安全校验 + 限速插值，安全地确认这条腿是否朝「站起来」方向收拢。「方向验证」
则让单腿各关节（或单个关节）从当前位置朝站立方向各转一个固定小角度（默认 10°），
用来快速判断哪个关节方向标反。
（此前的命令行脚本 `test_one_leg.py` 已被这两项网页功能取代并删除。）

```bash
python3 scripts/robot_web.py   # 浏览器开 http://127.0.0.1:8000
```

接线映射（已确认，可在 `config/joint_map.default.json` 改）：usb0=ID 1/2/3(FL)、
usb1=4/5/6(FR)、usb2=7/8/9(RL)、usb3=10/11/12(RR)，腿内顺序 hip/thigh/shank。

> 说明：电机为**单圈绝对值编码器**，绝对位置断电不变，**标定一次即可跨上电使用**；仅在
> 机械改动 / 重新装配 / 换电机 / 改 ID 后才需重新标定。站立前的安全校验（当前角是否贴近
> 配置趴姿）仍会执行，作为"确实趴好了"的安全门槛。

以下是 `motor_web.py`（电机管理网页）的功能、底层命令与 `sudo` 密码策略详解；
`robot_web.py` 与其共用同一套底层命令与密码策略。

## motor_web.py（电机管理网页）详解

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
|            | 读取所有 / 单个电机角度    | `motor_ctrl <port> <id|all> read`                 |
| 状态栏     | 取消当前命令               | 向当前子进程发 SIGTERM / SIGKILL                  |

- 扫描会自动把检测到的 ID 填进「原 ID」「驱动电机 ID」下拉框。
- 「切回电机模式」遇到无固件电机可能久等，可点「取消当前命令」中止。
- 「停止」按钮先杀掉驱动子进程，再补一段 500 ms 的 `mode=0` 停止脉冲。

## 依赖

- Linux + Python 3（纯标准库，无需额外依赖）
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

产物 `build/motor_ctrl` 必须存在，否则网页启动时会在输出区警告，「驱动转动」按钮不可用。

参数：

```
motor_ctrl <port> <id> drive [speed_rad_s] [duration_ms]
motor_ctrl <port> <id> stop  [duration_ms]
motor_ctrl <port> <id|all> read
```

- `duration_ms = 0`：永远循环，直到被外部 SIGTERM
- 收到 SIGTERM/SIGINT 时，若处于 drive 模式会自动补发 200 ms 的停止脉冲
- `read`：只发零力矩指令读取当前状态，**不驱动电机**；`all` 遍历 ID 1~12（本机电机从 1 起编号）。
  每个响应电机打印一行 `ANGLE id=.. ok=1 rotor=.. joint=.. deg=.. temp=.. err=..`，
  其中 `joint = rotor / 6.33`（减速比），即输出轴（关节）角度。

## 路径配置

脚本通过 `SDK_ROOT` 常量定位 SDK；优先读环境变量 `UNITREE_MOTOR_SDK`，
未设置时回退到一个默认硬编码路径（见脚本顶部）。

如果 SDK 位置变动，建议：

```bash
export UNITREE_MOTOR_SDK="/your/path/to/Linux平台电机使用例程(包含SDK)"
python3 scripts/motor_web.py
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
python3 scripts/motor_web.py   # 浏览器开 http://127.0.0.1:8000
```

脚本权限为 `700`，仅本用户可读 / 执行（因内置明文 sudo 密码，见下）。

## 权限与密码

`swboot` / `changeid` / `swmotor` / `motor_ctrl` 都需要 root 才能访问串口。
脚本顶部 `SUDO_PASSWORD = "1"` 用于通过 `sudo -S` 自动喂密码，免去每次输入。

> ⚠️ 这是明文密码。脚本权限已限制为 `700`（`chmod 700 motor_web.py robot_web.py`），
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
2. 启动网页，确认串口下拉框列出了 `/dev/ttyUSB0..3`。
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
├── motor_web.py        # 电机管理网页：ID 管理 / 驱动 / 读角度 / 姿态标定
├── robot_web.py        # 四足操控网页：整机站立 + 单腿验证 + 方向验证
└── README.md           # 本文档
```

SDK 目录侧的新增 / 修改：

```
<SDK_ROOT>/
├── example/motor_ctrl.cpp   # 新增：可传参的驱动 / 停止工具源码
├── CMakeLists.txt           # 新增 motor_ctrl target
└── build/motor_ctrl         # 编译产物
```
