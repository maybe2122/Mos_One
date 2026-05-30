#include "serialPort/SerialPort.h"
#include <unistd.h>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>

// GO-M8010-6 位置控制示例：把电机从【当前位置】平滑转动指定角度后保持。
// 低速也丝滑的原理：力矩主要来自 K_P×位置误差（编码器位置干净），而不是
// 低速时又糙又跳的速度估计；目标位置按轨迹平滑递增，运动时给速度前馈，
// 这样 K_W 阻尼项不会与运动对抗。
//
// 电机内部力矩 = K_P×(Pos目标 − Pos实际) + K_W×(W目标 − W实际) + T  （均为转子侧）

static const double PI   = 3.14159265358979323846;
static const double GEAR = 6.33;          // 减速比：转子角 = 关节角 × 6.33

static std::atomic<bool> g_stop(false);
static void on_sigint(int) { g_stop = true; }

int main() {
  // ===================== 可调参数 =====================
  const int    ID        = 2;       // 电机 ID
  const double MOVE_DEG  = 90.0;    // 要转动的【输出端关节角】(°)，正=正转 负=反转
  const double JOINT_VEL = 0.5;     // 关节角速度 (rad/s)，决定多快走完——越小越慢越稳
  const double K_P       = 2.0;     // 位置环刚度，越大越"硬"跟得越紧（抖就调小）
  const double K_W       = 0.05;     // 速度阻尼，抑制抖动（嗡鸣就调小）
  const double T_FF      = 0.0;     // 力矩前馈（转子 N·m，可正负），常驻施加，补偿重力/负载
  const double RATE_HZ   = 50.0;   // 控制频率
  // ====================================================

  std::signal(SIGINT, on_sigint);
  std::signal(SIGTERM, on_sigint);

  SerialPort serial("/dev/ttyUSB0");
  MotorCmd   cmd;
  MotorData  data;
  cmd.motorType = MotorType::GO_M8010_6;

  // 1) 先读当前转子位置当轨迹起点（K_P=K_W=0 只读不驱动）
  double cur = 0.0;
  bool got = false;
  for (int i = 0; i < 50 && !got; ++i) {
    cmd.id = ID; cmd.mode = 1;
    cmd.K_P = 0.0; cmd.K_W = 0.0; cmd.Pos = 0.0; cmd.W = 0.0; cmd.T = 0.0;
    serial.sendRecv(&cmd, &data);
    if (data.correct) { cur = data.Pos; got = true; }
    usleep(2000);
  }
  if (!got) {
    std::printf("读不到电机 id=%d：检查串口 / ID / 是否电机模式。\n", ID);
    return 1;
  }

  // 2) 目标 = 当前 + 角度增量（转子 rad）
  const double delta_rotor = MOVE_DEG * PI / 180.0 * GEAR;
  const double target      = cur + delta_rotor;
  const double vel_rotor   = JOINT_VEL * GEAR;          // 转子角速度 (rad/s)
  const double dir         = (delta_rotor >= 0) ? 1.0 : -1.0;
  const long   us          = (long)(1e6 / RATE_HZ);
  double pos_des = cur;

  std::printf("[位置控制] id=%d 起点=%.3f -> 目标=%.3f rad(转子) | 关节 %+.1f° | 速度 %.2f rad/s\n",
              ID, cur, target, MOVE_DEG, JOINT_VEL);
  std::fflush(stdout);

  // 3) 平滑递增设定点跑位置环；到点后保持位置（Ctrl+C 退出）。
  //    用【实测 dt】推进设定点，使设定点速率与速度前馈 cmd.W 严格一致；
  //    否则 sendRecv 阻塞导致实际循环慢于 RATE_HZ，前馈速度 > 设定点真实推进速度，
  //    电机"想跑快又被位置项拉回"来回较劲 → 抖动。
  long tick = 0;
  long t_prev = 0;
  {
    using namespace std::chrono;
    t_prev = duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
  }
  while (!g_stop.load()) {
    long tnow;
    {
      using namespace std::chrono;
      tnow = duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
    }
    double dt = (tnow - t_prev) / 1000.0;
    t_prev = tnow;
    if (dt <= 0.0 || dt > 0.1) dt = 1.0 / RATE_HZ;   // 防御异常 dt

    double stepmax = vel_rotor * dt;                 // 本周期最多推进的转子角
    bool arrived = std::fabs(target - pos_des) <= stepmax;
    if (arrived) pos_des = target;
    else         pos_des += dir * stepmax;

    cmd.id  = ID;
    cmd.mode = 1;
    cmd.K_P = K_P;
    cmd.K_W = K_W;
    cmd.Pos = pos_des;
    cmd.W   = arrived ? 0.0 : dir * vel_rotor;   // 运动中给速度前馈，到点归零→平滑保持
    cmd.T   = T_FF;                              // 力矩前馈（常驻）
    serial.sendRecv(&cmd, &data);

    if (++tick % 50 == 0) {                       // ~10Hz 打印，避免刷屏
      std::printf("Pos目标=%.3f 实际=%.3f W=%.3f T=%.3f Temp=%d Err=%d ok=%d\n",
                  pos_des, data.Pos, data.W, data.T,
                  (int)data.Temp, (int)data.MError, data.correct ? 1 : 0);
      std::fflush(stdout);
    }
    usleep(us);
  }

  // 4) 退出前发一小段刹车(mode=0)，让电机松力，避免一直保持力矩
  std::printf("\n[位置控制] 退出，松力 ...\n");
  for (int i = 0; i < 100; ++i) {
    cmd.id = ID; cmd.mode = 0;
    cmd.K_P = 0.0; cmd.K_W = 0.0; cmd.Pos = 0.0; cmd.W = 0.0; cmd.T = 0.0;
    serial.sendRecv(&cmd, &data);
    usleep(2000);
  }
  return 0;
}
