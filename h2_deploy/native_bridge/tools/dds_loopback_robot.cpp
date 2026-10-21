// DDS loopback robot for testing h2_udp_bridge without hardware.
// Publishes unitree_hg LowState with motor_state[i].q = 0.01 * (i + 1) and
// prints the first LowCmd it receives whose kp is non-zero, as one JSON line:
//   {"q":[...35], "kp":[...35], "kd":[...35]}
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

using LowCmd = unitree_hg::msg::dds_::LowCmd_;
using LowState = unitree_hg::msg::dds_::LowState_;

static uint32_t crc32_core(uint32_t * ptr, uint32_t len)
{
  uint32_t xbit = 0, data = 0, crc = 0xFFFFFFFF;
  const uint32_t poly = 0x04c11db7;
  for (uint32_t i = 0; i < len; i++) {
    xbit = 1u << 31;
    data = ptr[i];
    for (uint32_t bits = 0; bits < 32; bits++) {
      if (crc & 0x80000000) {
        crc <<= 1;
        crc ^= poly;
      } else {
        crc <<= 1;
      }
      if (data & xbit) crc ^= poly;
      xbit >>= 1;
    }
  }
  return crc;
}

int main(int argc, char ** argv)
{
  const std::string net = argc > 1 ? argv[1] : "lo";
  const double seconds = argc > 2 ? std::stod(argv[2]) : 10.0;
  unitree::robot::ChannelFactory::Instance()->Init(0, net);
  unitree::robot::ChannelPublisher<LowState> state_pub("rt/lowstate");
  state_pub.InitChannel();
  std::atomic<bool> done{false};
  std::mutex out;
  unitree::robot::ChannelSubscriber<LowCmd> cmd_sub("rt/lowcmd");
  cmd_sub.InitChannel([&](const void * msg) {
    const LowCmd cmd = *(const LowCmd *)msg;
    if (cmd.crc() != crc32_core((uint32_t *)&cmd, (sizeof(LowCmd) >> 2) - 1)) return;
    bool active = false;
    for (const auto & m : cmd.motor_cmd()) active = active || m.kp() != 0.0f;
    if (!active || done.exchange(true)) return;
    std::lock_guard<std::mutex> lock(out);
    auto dump = [&](const char * key, auto get) {
      std::cout << "\"" << key << "\":[";
      for (size_t i = 0; i < cmd.motor_cmd().size(); ++i) std::cout << (i ? "," : "") << get(cmd.motor_cmd()[i]);
      std::cout << "]";
    };
    std::cout << "LOWCMD {";
    dump("q", [](const auto & m) { return m.q(); });
    std::cout << ",";
    dump("kp", [](const auto & m) { return m.kp(); });
    std::cout << ",";
    dump("kd", [](const auto & m) { return m.kd(); });
    std::cout << "}" << std::endl;
  }, 1);
  LowState state;
  uint32_t tick = 0;
  const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
  while (std::chrono::steady_clock::now() < end) {
    state.tick() = ++tick;
    state.mode_machine() = 7;
    for (size_t i = 0; i < state.motor_state().size(); ++i) {
      state.motor_state()[i].q() = 0.01f * static_cast<float>(i + 1);
      state.motor_state()[i].dq() = -0.01f * static_cast<float>(i + 1);
    }
    state.imu_state().quaternion() = {1.0f, 0.0f, 0.0f, 0.0f};
    state.crc() = crc32_core((uint32_t *)&state, (sizeof(LowState) >> 2) - 1);
    state_pub.Write(state);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return done ? 0 : 2;
}
