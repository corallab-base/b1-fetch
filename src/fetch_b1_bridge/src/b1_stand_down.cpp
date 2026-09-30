// One-shot: ask the B1 to lie down.
//
//   mode 6 (position stand)      - leave balance stand / walking; the robot
//                                  ignores stand-down from balance stand
//                                  (reported mode 1), obeys it from here
//   mode 5 (position stand down) - lower to the ground
//
// Not a ROS node. Asks for confirmation, prints what it sends, the robot's
// reported mode and body height, then stops sending. Ctrl+C stops sending
// immediately. Judge progress by height: the reported mode does not mirror
// the commanded mode.
//
// Stop b1_bridge / ros_control first: only one program should command the
// robot (nothing here detects a second sender).
//
// usage: b1_stand_down [down_seconds=6] [target_ip=192.168.123.220]

#include "unitree_legged_sdk/unitree_legged_sdk.h"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
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
  const double down_seconds = argc > 1 ? std::atof(argv[1]) : 6.0;
  const char* target_ip = argc > 2 ? argv[2] : "192.168.123.220";
  const Phase phases[] = {
      {6, 2.0, "position stand (leave balance stand)"},
      {5, down_seconds, "position stand down"},
  };

  std::printf("Will lie the robot down at %s:8082: mode 6 (2 s) -> 5 (%.1f s).\n"
              "Stop b1_bridge / ros_control first. Clear the area under the robot.\n"
              "Type 'yes' to send: ", target_ip, down_seconds);
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
