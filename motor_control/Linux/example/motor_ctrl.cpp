// motor_ctrl.cpp
// 可传参的 GO-M8010-6 电机驱动 / 停止 / 读取小工具，给 GUI 使用。
// 用法：
//   motor_ctrl <port> <id> drive [speed_rad_s] [duration_ms]
//   motor_ctrl <port> <id> stop  [duration_ms]
//   motor_ctrl <port> <id|all> read
// 参数说明：
//   port            串口号，如 /dev/ttyUSB0
//   id              电机 ID；read 时可用 all 遍历 0~14
//   speed_rad_s     目标输出转速（rad/s），默认 6.28*6.33（与 main.cpp 一致）
//   duration_ms     持续时间（毫秒），0 = 永远（直到外部 kill），默认 drive=0 / stop=500
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
#include <string>
#include <unistd.h>

// GO-M8010-6 减速比：data.Pos 是转子角度，关节角度 = 转子角度 / 6.33。
static const double GEAR_RATIO = 6.33;

static std::atomic<bool> g_stop_requested(false);

static void on_signal(int) { g_stop_requested.store(true); }

static void print_usage(const char *argv0) {
    std::fprintf(stderr,
        "Usage:\n"
        "  %s <port> <id> drive [speed_rad_s] [duration_ms]\n"
        "  %s <port> <id> stop  [duration_ms]\n"
        "  %s <port> <id|all> read\n",
        argv0, argv0, argv0);
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

int main(int argc, char **argv) {
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
