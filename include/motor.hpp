#pragma once
#include <string>
#include <mutex>
#include <optional>
#include <array>
#include "usb2can_transport.hpp"
#include "rs_protocol.hpp"
#include "actuator.hpp"
#include "stats.hpp"

struct PVTT { float p=0, v=0, t=0, temp=0; };

struct BusHandle {
  int dev_idx = 0;
  std::string dev_name;
  Usb2CanDevice* dev = nullptr;
  uint8_t channel = 1; // 1 or 2
};

class RobStrideMotor {
public:
  RobStrideMotor(BusHandle bus, uint8_t master_id, uint8_t motor_id, ActuatorType type,
                 std::string group, Stats* stats, int stats_idx);

  uint8_t motor_id() const { return motor_id_; }
  const BusHandle& bus() const { return bus_; }
  const std::string& group() const { return group_; }

  // --- basic commands ---
  void enable();
  void stop(uint8_t clear_error=0);
  void set_zero();
  void set_mode(uint8_t run_mode); // 写 0x7005
  void set_param_f32(uint16_t index, float value);
  void set_param_u8 (uint16_t index, uint8_t value);
  void get_param(uint16_t index);

  // --- motion control ---
  void send_motion_command(float torque, float pos_rad, float vel_rad_s, float kp, float kd);

  // --- rx hook (router调用) ---
  void on_frame(const Usb2CanFrame& f);

  std::optional<PVTT> last_pvtt() const;

private:
  BusHandle bus_;
  uint8_t master_id_;
  uint8_t motor_id_;
  ActuatorType type_;
  std::string group_;

  Stats* stats_ = nullptr;
  int stats_idx_ = -1;

  mutable std::mutex mu_;
  PVTT pvtt_{};
  bool has_pvtt_ = false;
};
