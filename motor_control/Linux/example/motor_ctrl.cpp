// motor_ctrl.cpp
// 可传参的 GO-M8010-6 电机驱动 / 停止 / 读取 / 流式位置伺服小工具，给 GUI / 网页使用。
// 用法：
//   motor_ctrl <port> <id> drive [speed_rad_s] [duration_ms]
//   motor_ctrl <port> <id> stop  [duration_ms]
//   motor_ctrl <port> <id|all> read
//   motor_ctrl <port> servo <kp> <kw>
// 参数说明：
//   port            串口号，如 /dev/ttyUSB0
//   id              电机 ID 1~12；read 时可用 all 遍历 1~12
//   speed_rad_s     目标【转子】转速(rad/s)，直接写入 cmd.W(手册表1 ω_set 为转子侧)；
//                   输出端转速 = 此值 / 6.33。默认 6.28*6.33(=输出 6.28rad/s，与 main.cpp 一致)
//   duration_ms     持续时间（毫秒），0 = 永远（直到外部 kill），默认 drive=0 / stop=500
//   servo kp kw     常驻流式位置伺服：从 stdin 逐行读 "<id> <pos_rotor> [<id> <pos_rotor>...]"，
//                   以 mode=1,K_P=kp,K_W=kw,Pos=target 持续重发（到位后保持）。
//                   收到一行 "stop" 或 EOF / SIGTERM 退出，退出前补 mode=0 停止脉冲。
//
// drive 退出时（不论被 kill 还是计时结束）发一段 mode=0 的停止脉冲，避免电机失控。
// read 只发送零力矩指令读取当前状态，不驱动电机，读完即退出。

#include "serialPort/SerialPort.h"
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

// GO-M8010-6 减速比：data.Pos 是转子角度，关节角度 = 转子角度 / 6.33。
static const double GEAR_RATIO = 6.33;

// 本机共 12 个电机，从 1 开始编号，合法单播 ID 为 1~12。
// （手册：ID 15 为广播地址，单播无返回；read all 在 [MIN,MAX] 区间内遍历。）
static const int MIN_MOTOR_ID = 1;
static const int MAX_MOTOR_ID = 12;

// 调试打印间隔(ms)：servo / moveto 每隔这么久打印一行【全部参数】。
// 调小（如 20）能看清震动细节，但日志更密；调大更安静。
static long PRINT_INTERVAL_MS = 200;

static std::atomic<bool> g_stop_requested(false);

static void on_signal(int) { g_stop_requested.store(true); }

static void print_usage(const char *argv0) {
    std::fprintf(stderr,
        "Usage:\n"
        "  %s <port> <id> drive  [speed_rad_s] [duration_ms]\n"
        "  %s <port> <id> stop   [duration_ms]\n"
        "  %s <port> <id|all> read\n"
        "  %s <port> <id> moveto <move_deg> <joint_vel> [kp] [kw] [t_ff] [on_arrive:release|hold|keep]\n"
        "  %s <port> servo <kp> <kw>\n",
        argv0, argv0, argv0, argv0, argv0);
}

// 读取单个电机的当前角度并打印一行可被 GUI 解析的结果。
// 用 mode=1 + 增益/目标全 0 => 不输出力矩，只读取，不会让电机转动。
// 首帧可能无响应，最多重试若干次直到 data.correct。
static void read_one(SerialPort &serial, int id) {
    MotorCmd cmd;
    MotorData data;
    cmd.motorType = MotorType::GO_M8010_6;
    cmd.id  = id;
    cmd.mode = 1;
    cmd.K_P = 0.0;
    cmd.K_W = 0.0;
    cmd.Pos = 0.0;
    cmd.W   = 0.0;
    cmd.T   = 0.0;

    bool got = false;
    for (int attempt = 0; attempt < 10 && !got; ++attempt) {
        serial.sendRecv(&cmd, &data);
        if (data.correct) {
            got = true;
            break;
        }
        usleep(2000);
    }

    if (got) {
        double joint = data.Pos / GEAR_RATIO;
        std::printf("ANGLE id=%d ok=1 rotor=%.6f joint=%.6f deg=%.3f temp=%d err=%d\n",
                    id, data.Pos, joint, joint * 180.0 / M_PI,
                    (int)data.Temp, (int)data.MError);
    } else {
        std::printf("ANGLE id=%d ok=0\n", id);
    }
    std::fflush(stdout);
}

static long now_ms() {
    using namespace std::chrono;
    return duration_cast<milliseconds>(
               steady_clock::now().time_since_epoch())
        .count();
}

// 流式位置伺服：常驻进程，按总线持续以位置环重发各电机目标。
// stdin 每行: "<id> <pos_rotor> [<id> <pos_rotor> ...]"，更新（并新增）目标电机。
// 一行 "stop" / EOF / SIGTERM 退出；退出前对所有出现过的电机补 mode=0 停止脉冲。
// 关节限速 / 插值由上层（Python）负责，本进程只把收到的目标位置交给电机内部 PD 跟随。
static std::mutex g_mtx;
static std::map<int, double> g_targets;     // id -> 目标转子角(rad)
static std::vector<int> g_order;            // 出现顺序，便于稳定遍历
static std::atomic<bool> g_eof(false);

static void servo_stdin_reader() {
    std::string line;
    while (std::getline(std::cin, line)) {
        // 去掉首尾空白
        size_t a = line.find_first_not_of(" \t\r\n");
        if (a == std::string::npos) continue;
        std::string s = line.substr(a);
        if (s.rfind("stop", 0) == 0) {        // "stop" 行 => 请求退出
            g_stop_requested.store(true);
            return;
        }
        std::istringstream iss(s);
        int id;
        double pos;
        std::lock_guard<std::mutex> lk(g_mtx);
        while (iss >> id >> pos) {
            if (id < MIN_MOTOR_ID || id > MAX_MOTOR_ID) continue;
            if (g_targets.find(id) == g_targets.end()) g_order.push_back(id);
            g_targets[id] = pos;
        }
    }
    g_eof.store(true);                        // stdin 关闭
}

static int run_servo(const std::string &port, double kp, double kw) {
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    SerialPort serial(port);
    MotorCmd cmd;
    MotorData data;
    cmd.motorType = MotorType::GO_M8010_6;

    std::thread reader(servo_stdin_reader);
    reader.detach();

    std::printf("[motor_ctrl] servo start port=%s kp=%.3f kw=%.3f\n",
                port.c_str(), kp, kw);
    std::fflush(stdout);

    long last_print = now_ms();
    while (!g_stop_requested.load()) {
        // 取一份当前目标快照，逐个电机发位置指令
        std::vector<int> ids;
        std::map<int, double> tgt;
        {
            std::lock_guard<std::mutex> lk(g_mtx);
            ids = g_order;
            tgt = g_targets;
        }
        for (int id : ids) {
            cmd.id = id;
            cmd.mode = 1;
            cmd.K_P = kp;
            cmd.K_W = kw;
            cmd.Pos = tgt[id];
            cmd.W = 0.0;
            cmd.T = 0.0;
            serial.sendRecv(&cmd, &data);
            long t = now_ms();
            if (t - last_print >= PRINT_INTERVAL_MS) {
                double err = cmd.Pos - data.Pos;   // 目标 − 反馈（转子角 rad）
                std::printf("servo id=%d mode=%d K_P=%.3f K_W=%.3f"
                            " | 指令 Pos=%.4f W=%.3f T=%.3f"
                            " | 反馈 Pos=%.4f W=%.3f T=%.3f Temp=%d MError=%d ok=%d"
                            " | 误差 %.4f rad = %.2f°(关节)\n",
                            id, cmd.mode, cmd.K_P, cmd.K_W,
                            cmd.Pos, cmd.W, cmd.T,
                            data.Pos, data.W, data.T, (int)data.Temp,
                            (int)data.MError, data.correct ? 1 : 0,
                            err, err / GEAR_RATIO * 180.0 / M_PI);
                // 机器可解析的紧凑反馈行（供 robot_web.py 实时表格用；前缀 FB，不进日志）
                std::printf("FB id=%d pos=%.5f vel=%.4f tau=%.4f temp=%d merr=%d ok=%d errd=%.3f\n",
                            id, data.Pos, data.W, data.T, (int)data.Temp,
                            (int)data.MError, data.correct ? 1 : 0,
                            err / GEAR_RATIO * 180.0 / M_PI);
                std::fflush(stdout);
            }
        }
        long t = now_ms();
        if (t - last_print >= PRINT_INTERVAL_MS) last_print = t;
        if (ids.empty() && g_eof.load()) break;   // 还没收到任何目标且 stdin 已关 => 退出
        usleep(2000);                              // ~ 每轮 2ms，整体由 sendRecv 节流
    }

    // 退出前：对所有出现过的电机补一段 mode=0 停止脉冲
    std::vector<int> ids;
    {
        std::lock_guard<std::mutex> lk(g_mtx);
        ids = g_order;
    }
    if (!ids.empty()) {
        std::printf("[motor_ctrl] servo stop pulse for %zu motor(s)...\n", ids.size());
        std::fflush(stdout);
        long stop_start = now_ms();
        while (now_ms() - stop_start < 200) {
            for (int id : ids) {
                cmd.id = id;
                cmd.mode = 0;
                cmd.K_P = 0.0;
                cmd.K_W = 0.0;
                cmd.Pos = 0.0;
                cmd.W = 0.0;
                cmd.T = 0.0;
                serial.sendRecv(&cmd, &data);
            }
            usleep(2000);
        }
    }
    std::printf("[motor_ctrl] servo exit.\n");
    return 0;
}

// 平滑位置移动（与 example/main.cpp 的位置控制逻辑一致）：
// 先读当前转子位置当起点，再把目标位置按 joint_vel 平滑递增（轨迹），
// 以 mode=1 + K_P/K_W + 速度前馈跟随，到位后保持；收到 SIGTERM/SIGINT 退出前松力。
// 低速也丝滑的原因：力矩主要来自 K_P×位置误差（编码器位置干净），目标平滑递增，
// 运动时给速度前馈，K_W 阻尼项不与运动对抗。
static int run_moveto(const std::string &port, int id, double move_deg,
                      double joint_vel, double kp, double kw, double t_ff,
                      const std::string &on_arrive) {
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    SerialPort serial(port);
    MotorCmd cmd;
    MotorData data;
    cmd.motorType = MotorType::GO_M8010_6;

    // 1) 读当前转子位置当轨迹起点（K_P=K_W=0 只读不驱动）
    double cur = 0.0;
    bool got = false;
    for (int i = 0; i < 50 && !got; ++i) {
        cmd.id = id; cmd.mode = 1;
        cmd.K_P = 0.0; cmd.K_W = 0.0; cmd.Pos = 0.0; cmd.W = 0.0; cmd.T = 0.0;
        serial.sendRecv(&cmd, &data);
        if (data.correct) { cur = data.Pos; got = true; }
        usleep(2000);
    }
    if (!got) {
        std::printf("MOVETO id=%d ok=0 (读不到当前位置：检查串口/ID/电机模式)\n", id);
        std::fflush(stdout);
        return 1;
    }

    const double RATE_HZ = 500.0;
    const double delta_rotor = move_deg * M_PI / 180.0 * GEAR_RATIO;
    const double target = cur + delta_rotor;
    const double vel_rotor = joint_vel * GEAR_RATIO;     // 转子角速度
    const double dir = (delta_rotor >= 0) ? 1.0 : -1.0;
    const long us = (long)(1e6 / RATE_HZ);
    double pos_des = cur;

    std::printf("[motor_ctrl] moveto id=%d cur=%.3f -> target=%.3f rad(rotor) "
                "| joint %+.1f deg | vel=%.2f rad/s kp=%.2f kw=%.2f t_ff=%.3f on_arrive=%s\n",
                id, cur, target, move_deg, joint_vel, kp, kw, t_ff, on_arrive.c_str());
    std::fflush(stdout);

    // 2) 平滑递增设定点跑位置环。用【实测 dt】推进设定点，使设定点速率与速度前馈
    //    严格一致——否则 sendRecv 阻塞导致实际循环慢于 RATE_HZ，前馈速度会大于设定点
    //    真实推进速度，电机"想跑快又被位置项拉回"来回较劲 → 抖动。
    //    到位后按 on_arrive 处理：hold=持续保持(直到停止信号)；release/keep=稳定后退出。
    const long SETTLE_MS = 300;        // 到位后再稳定多久才退出
    bool arrived_latched = false;
    long arrived_time = 0;
    long last_print = now_ms();
    long t_prev = now_ms();
    while (!g_stop_requested.load()) {
        long tnow = now_ms();
        double dt = (tnow - t_prev) / 1000.0;
        t_prev = tnow;
        if (dt <= 0.0 || dt > 0.1) dt = 1.0 / RATE_HZ;   // 防御异常 dt

        double stepmax = vel_rotor * dt;                 // 本周期最多推进的转子角
        bool arrived = std::fabs(target - pos_des) <= stepmax;
        if (arrived) pos_des = target;
        else         pos_des += dir * stepmax;

        cmd.id = id; cmd.mode = 1;
        cmd.K_P = kp; cmd.K_W = kw;
        cmd.Pos = pos_des;
        cmd.W = arrived ? 0.0 : dir * vel_rotor;   // 与设定点速率一致的速度前馈，到位归零
        cmd.T = t_ff;                              // 力矩前馈（常驻，可补偿重力/负载）
        serial.sendRecv(&cmd, &data);

        if (tnow - last_print >= PRINT_INTERVAL_MS) {
            last_print = tnow;
            double err = cmd.Pos - data.Pos;   // 目标 − 反馈（转子角 rad）
            std::printf("moveto id=%d mode=%d K_P=%.3f K_W=%.3f"
                        " | 指令 Pos=%.4f W=%.3f T=%.3f"
                        " | 反馈 Pos=%.4f W=%.3f T=%.3f Temp=%d MError=%d ok=%d"
                        " | 误差 %.4f rad = %.2f°(关节)%s\n",
                        id, cmd.mode, cmd.K_P, cmd.K_W,
                        cmd.Pos, cmd.W, cmd.T,
                        data.Pos, data.W, data.T, (int)data.Temp,
                        (int)data.MError, data.correct ? 1 : 0,
                        err, err / GEAR_RATIO * 180.0 / M_PI,
                        arrived ? " [到位]" : "");
            std::fflush(stdout);
        }

        // 到位后：hold 模式继续保持；release/keep 稳定 SETTLE_MS 后退出循环
        if (arrived && on_arrive != "hold") {
            if (!arrived_latched) { arrived_latched = true; arrived_time = tnow; }
            if (tnow - arrived_time >= SETTLE_MS) break;
        }
        usleep(us);
    }

    // 3) 退出处理：
    //    - 被外部停止(SIGTERM) 或 on_arrive=release：发 mode=0 松力脉冲（电机松开）
    //    - on_arrive=keep：不发松力，保留最后一帧（电机维持最后命令，固件相关）
    bool brake = g_stop_requested.load() || (on_arrive != "keep");
    if (brake) {
        std::printf("[motor_ctrl] moveto exit: releasing (mode=0)...\n");
        std::fflush(stdout);
        long stop_start = now_ms();
        while (now_ms() - stop_start < 200) {
            cmd.id = id; cmd.mode = 0;
            cmd.K_P = 0.0; cmd.K_W = 0.0; cmd.Pos = 0.0; cmd.W = 0.0; cmd.T = 0.0;
            serial.sendRecv(&cmd, &data);
            usleep(2000);
        }
    } else {
        std::printf("[motor_ctrl] moveto exit: keep last command (no brake).\n");
        std::fflush(stdout);
    }
    std::printf("[motor_ctrl] moveto exit.\n");
    return 0;
}

int main(int argc, char **argv) {
    // 流式位置伺服：motor_ctrl <port> servo <kp> <kw>（action 在 argv[2]，无 id 位）
    if (argc >= 3 && std::string(argv[2]) == "servo") {
        if (argc < 5) {
            print_usage(argv[0]);
            return 1;
        }
        double kp = std::atof(argv[3]);
        double kw = std::atof(argv[4]);
        return run_servo(argv[1], kp, kw);
    }

    if (argc < 4) {
        print_usage(argv[0]);
        return 1;
    }

    std::string port = argv[1];
    std::string id_arg = argv[2];
    std::string action = argv[3];

    bool read_all = (action == "read" && id_arg == "all");
    int id = std::atoi(id_arg.c_str());

    if (!read_all && (id < MIN_MOTOR_ID || id > MAX_MOTOR_ID)) {
        std::fprintf(stderr, "id 必须在 %d~%d 范围(read 时可用 all): %s\n",
                     MIN_MOTOR_ID, MAX_MOTOR_ID, id_arg.c_str());
        return 2;
    }

    // 纯读取：遍历(all)或单个 ID，打印当前角度后立即退出。不驱动电机。
    if (action == "read") {
        SerialPort serial(port);
        if (read_all) {
            for (int i = MIN_MOTOR_ID; i <= MAX_MOTOR_ID; ++i) read_one(serial, i);
        } else {
            read_one(serial, id);
        }
        return 0;
    }

    // 平滑位置移动：moveto <move_deg> <joint_vel> [kp] [kw]
    if (action == "moveto") {
        if (argc < 6) {
            print_usage(argv[0]);
            return 1;
        }
        double move_deg  = std::atof(argv[4]);
        double joint_vel = std::atof(argv[5]);
        double kp   = (argc >= 7) ? std::atof(argv[6]) : 2.0;
        double kw   = (argc >= 8) ? std::atof(argv[7]) : 0.1;
        double t_ff = (argc >= 9) ? std::atof(argv[8]) : 0.0;
        std::string on_arrive = (argc >= 10) ? std::string(argv[9]) : "release";
        return run_moveto(port, id, move_deg, joint_vel, kp, kw, t_ff, on_arrive);
    }

    double speed = 6.28 * 6.33;
    long duration_ms = 0;

    if (action == "drive") {
        if (argc >= 5) speed = std::atof(argv[4]);
        if (argc >= 6) duration_ms = std::atol(argv[5]);
    } else if (action == "stop") {
        if (argc >= 5) duration_ms = std::atol(argv[4]);
        if (duration_ms <= 0) duration_ms = 500;
    } else {
        print_usage(argv[0]);
        return 1;
    }

    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    SerialPort serial(port);
    MotorCmd cmd;
    MotorData data;

    cmd.motorType = MotorType::GO_M8010_6;
    cmd.id = id;
    cmd.K_P = 0.0;
    cmd.K_W = 0.05;
    cmd.Pos = 0.0;
    cmd.T = 0.0;

    const long start = now_ms();
    long last_print = start;

    std::printf("[motor_ctrl] port=%s id=%d action=%s speed=%.3f duration_ms=%ld\n",
                port.c_str(), id, action.c_str(), speed, duration_ms);
    std::fflush(stdout);

    while (!g_stop_requested.load()) {
        if (action == "drive") {
            cmd.mode = 1;
            cmd.W = speed;
        } else {
            cmd.mode = 0;
            cmd.W = 0.0;
        }
        serial.sendRecv(&cmd, &data);

        long t = now_ms();
        if (t - last_print >= 200) {
            last_print = t;
            if (data.correct) {
                std::printf("motor id=%d Pos=%.3f W=%.3f T=%.3f Temp=%d MError=%d\n",
                            id, data.Pos, data.W, data.T,
                            (int)data.Temp, (int)data.MError);
            } else {
                std::printf("motor id=%d no response\n", id);
            }
            std::fflush(stdout);
        }

        if (duration_ms > 0 && (t - start) >= duration_ms) break;

        usleep(200);
    }

    // 退出前发送一小段停止脉冲，避免 drive 模式异常退出后电机继续转
    if (action == "drive") {
        std::printf("[motor_ctrl] sending stop pulse...\n");
        std::fflush(stdout);
        long stop_start = now_ms();
        cmd.mode = 0;
        cmd.W = 0.0;
        while (now_ms() - stop_start < 200) {
            serial.sendRecv(&cmd, &data);
            usleep(200);
        }
    }

    std::printf("[motor_ctrl] exit.\n");
    return 0;
}
