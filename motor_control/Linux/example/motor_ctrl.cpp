// motor_ctrl.cpp
// 可传参的 GO-M8010-6 电机驱动 / 停止 / 读取 / 流式位置伺服小工具，给 GUI / 网页使用。
// 用法：
//   motor_ctrl <port> <id> drive [speed_rad_s] [duration_ms]
//   motor_ctrl <port> <id> stop  [duration_ms]
//   motor_ctrl <port> <id|all> read
//   motor_ctrl <port> servo <kp> <kw>
// 参数说明：
//   port            串口号，如 /dev/ttyUSB0
//   id              电机 ID；read 时可用 all 遍历 0~14
//   speed_rad_s     目标输出转速（rad/s），默认 6.28*6.33（与 main.cpp 一致）
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

static std::atomic<bool> g_stop_requested(false);

static void on_signal(int) { g_stop_requested.store(true); }

static void print_usage(const char *argv0) {
    std::fprintf(stderr,
        "Usage:\n"
        "  %s <port> <id> drive [speed_rad_s] [duration_ms]\n"
        "  %s <port> <id> stop  [duration_ms]\n"
        "  %s <port> <id|all> read\n"
        "  %s <port> servo <kp> <kw>\n",
        argv0, argv0, argv0, argv0);
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
            if (id < 0 || id > 15) continue;
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
            if (t - last_print >= 200) {
                std::printf("servo id=%d tgt=%.3f Pos=%.3f T=%.3f Temp=%d MError=%d ok=%d\n",
                            id, tgt[id], data.Pos, data.T, (int)data.Temp,
                            (int)data.MError, data.correct ? 1 : 0);
                std::fflush(stdout);
            }
        }
        long t = now_ms();
        if (t - last_print >= 200) last_print = t;
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

    if (!read_all && (id < 0 || id > 15)) {
        std::fprintf(stderr, "id 必须在 0~15 范围(read 时可用 all): %s\n", id_arg.c_str());
        return 2;
    }

    // 纯读取：遍历(all)或单个 ID，打印当前角度后立即退出。不驱动电机。
    if (action == "read") {
        SerialPort serial(port);
        if (read_all) {
            for (int i = 0; i <= 14; ++i) read_one(serial, i);
        } else {
            read_one(serial, id);
        }
        return 0;
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
