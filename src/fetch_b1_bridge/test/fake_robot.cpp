// Fake B1 high-level controller for localhost bridge tests. Listens on
// 127.0.0.1:8082, replies to each command with a HighState (or streams at
// 500 Hz with --stream). Reads control lines from a FIFO/file polled every
// 10 ms:  mode N | height H | btn HEX | lx F | mute 0/1
// Logs every received command as: t_ms mode vx vy wz
// Walks like the real B1: mode 2 latches the velocity until the next mode-2
// command, idle modes don't stop it. Reports position, velocity and IMU
// attitude like the real robot's own odometry.
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>
#include <thread>
#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/socket.h>
using namespace UNITREE_LEGGED_SDK;
namespace UNITREE_LEGGED_SDK { uint32_t crc32(uint32_t*, uint32_t); }

int main(int argc, char** argv) {
  const char* ctl = argc > 1 ? argv[1] : "ctl";
  bool stream = argc > 2 && std::string(argv[2]) == "--stream";
  // raw socket: the SDK's SetSend(char*) sends zeros. The bridge's SDK accepts
  // a plain crc32 over all words but the last (checked with crc_probe).
  int fd = socket(AF_INET, SOCK_DGRAM, 0);
  sockaddr_in me{}, to{};
  me.sin_family = to.sin_family = AF_INET;
  me.sin_port = htons(8082); to.sin_port = htons(8090);
  inet_pton(AF_INET, "127.0.0.1", &me.sin_addr); inet_pton(AF_INET, "127.0.0.1", &to.sin_addr);
  if (bind(fd, (sockaddr*)&me, sizeof(me)) != 0) { perror("bind"); return 1; }
  fcntl(fd, F_SETFL, O_NONBLOCK);
  HighState st = {0};
  st.head = {0xFE, 0xEF};
  st.levelFlag = HIGHLEVEL;
  st.mode = 1;
  st.bodyHeight = 0.55f;
  st.imu.quaternion = {1, 0, 0, 0};
  double yaw = 0.0;
  auto last_step = std::chrono::steady_clock::now();
  bool mute = false;
  xRockerBtnDataStruct keys;
  std::memset(&keys, 0, sizeof(keys));
  auto t0 = std::chrono::steady_clock::now();
  auto next_ctl = t0;
  size_t ctl_pos = 0;
  HighCmd cmd = {0};
  double last_rx_ms = -1e9;
  std::string last_line;
  for (;;) {
    auto now = std::chrono::steady_clock::now();
    double t_ms = std::chrono::duration<double, std::milli>(now - t0).count();
    if (now >= next_ctl) {
      next_ctl = now + std::chrono::milliseconds(10);
      std::ifstream f(ctl);
      f.seekg(ctl_pos);
      std::string k; double v;
      while (f >> k) {
        if (k == "btn") { std::string h; f >> h; keys.btn.value = (uint16_t)std::stoul(h, nullptr, 16); }
        else if (f >> v) {
          if (k == "mode") st.mode = (uint8_t)v;
          else if (k == "height") st.bodyHeight = (float)v;
          else if (k == "lx") keys.lx = (float)v;
          else if (k == "mute") mute = v != 0;
          else if (k == "quit") return 0;
        }
        std::printf("%9.1f CTL %s\n", t_ms, k.c_str());
        ctl_pos = f.tellg() == std::streampos(-1) ? ctl_pos : (size_t)f.tellg();
      }
      if (f.eof()) { f.clear(); f.seekg(0, std::ios::end); ctl_pos = (size_t)f.tellg(); }
      std::memcpy(&st.wirelessRemote[0], &keys, sizeof(keys));
    }
    // ---- kinematics, as the real B1 behaved (2026-10-03): mode 2 latches its
    // velocity, which holds with no packets; idle modes are ignored while
    // walking, so only a mode-2 zero-velocity command stops it
    const double dt = std::chrono::duration<double>(now - last_step).count();
    last_step = now;
    st.position[0] += (std::cos(yaw) * st.velocity[0] - std::sin(yaw) * st.velocity[1]) * dt;
    st.position[1] += (std::sin(yaw) * st.velocity[0] + std::cos(yaw) * st.velocity[1]) * dt;
    yaw += st.yawSpeed * dt;
    st.imu.quaternion = {(float)std::cos(yaw / 2), 0, 0, (float)std::sin(yaw / 2)};
    st.imu.gyroscope[2] = st.yawSpeed;
    st.imu.accelerometer[2] = 9.81f;

    bool got = false;
    while (recv(fd, &cmd, sizeof(cmd), 0) > 0) got = true;  // newest command
    if (got && cmd.mode == 2 && st.mode <= 2) {
      st.mode = 2;
      st.velocity[0] = cmd.velocity[0];
      st.velocity[1] = cmd.velocity[1];
      st.yawSpeed = cmd.yawSpeed;
    }
    if (got) {
      char line[96];
      std::snprintf(line, sizeof(line), "mode=%u v=(%+.2f,%+.2f) wz=%+.2f", cmd.mode, cmd.velocity[0], cmd.velocity[1], cmd.yawSpeed);
      if (line != last_line || t_ms - last_rx_ms > 50) {  // log changes and resumes only
        std::printf("%9.1f RX %s%s\n", t_ms, line, t_ms - last_rx_ms > 50 ? "  (after gap)" : "");
        std::fflush(stdout);
        last_line = line;
      }
      last_rx_ms = t_ms;
    }
    if (last_rx_ms > 0 && t_ms - last_rx_ms > 50 && t_ms - last_rx_ms < 60) {
      std::printf("%9.1f RX stopped\n", last_rx_ms); std::fflush(stdout); last_rx_ms -= 20;  // print once
    }
    if (!mute && (got || stream)) {
      st.crc = crc32(reinterpret_cast<uint32_t*>(&st), sizeof(st) / 4 - 1);
      sendto(fd, &st, sizeof(st), 0, (sockaddr*)&to, sizeof(to));
    }
    std::this_thread::sleep_for(std::chrono::microseconds(stream ? 2000 : 200));
  }
}
