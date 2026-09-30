// One-shot: bring the B1 to balance stand, ready for velocity commands.
// Mirrors the order of Unitree's B1 example_walk.cpp:
//
//   mode 6 (position stand up)   - rise to standing height (holds if already up)
//   mode 1 (force/balance stand) - the state walking (mode 2) starts from
//
// Safe to run while already standing: no stand-down step, so no dip.
//
// Not a ROS node. Asks for confirmation, prints the robot's reported mode and
// body height, then stops sending. Ctrl+C stops sending immediately.
// Note: the robot's reported mode does not mirror the commanded mode (it
// reported 6 while obeying mode 5), so judge progress by body height.
//
// usage: b1_stand_up [target_ip=192.168.123.220]

#include "unitree_legged_sdk/unitree_legged_sdk.h"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <iostream>
#include <string>
#include <thread>

using namespace UNITREE_LEGGED_SDK;

static std::atomic<bool> g_stop(false);

struct Phase {
  uint8_t mode;
  double seconds;
  const char* what;
};

int main(int argc, char** argv) {
  const char* target_ip = argc > 1 ? argv[1] : "192.168.123.220";
  const Phase phases[] = {
      {6, 4.0, "position stand up"},
      {1, 2.0, "balance stand"},
  };

  std::printf("Will stand the robot up at %s:8082: mode 6 (4 s) -> 1 (2 s).\n"
              "Stop b1_bridge / ros_control first. Clear ~1 m around the robot.\n"
              "Type 'yes' to send: ", target_ip);
  std::string answer;
  std::getline(std::cin, answer);
  if (answer != "yes") {
    std::printf("Aborted, nothing sent.\n");
    return 0;
  }

  std::signal(SIGINT, [](int) { g_stop = true; });

  UDP udp(HIGHLEVEL, 8090, target_ip, 8082);
  HighCmd templ = {0};
  udp.InitCmdData(templ);
  HighState state = {0};

  const auto period = std::chrono::microseconds(2000);
  auto next = std::chrono::steady_clock::now();
  double t = 0.0;
  for (const Phase& p : phases) {
    if (g_stop) break;
    std::printf("-- mode %u: %s\n", p.mode, p.what);
    HighCmd cmd = templ;
    cmd.mode = p.mode;
    const int ticks = static_cast<int>(p.seconds / 0.002);
    for (int i = 0; i < ticks && !g_stop; ++i, t += 0.002) {
      udp.Recv();
      udp.GetRecv(state);
      udp.SetSend(cmd);
      udp.Send();
      if (i % 100 == 0) {
        std::printf("t=%4.1fs  sent mode=%u  robot mode=%u  height=%.3f\n", t, cmd.mode, state.mode,
                    state.bodyHeight);
        std::fflush(stdout);
      }
      next += period;
      std::this_thread::sleep_until(next);
    }
  }
  std::printf("%s. Stopped sending. Final robot mode=%u height=%.3f\n",
              g_stop ? "Interrupted" : "Done", state.mode, state.bodyHeight);
  return 0;
}
