#pragma once
#include <thread>
#include <atomic>
#include <vector>
#include <unordered_map>
#include <mutex>
#include "usb2can_transport.hpp"
#include "motor.hpp"

class Usb2CanRouter {
public:
  explicit Usb2CanRouter(const std::vector<Usb2CanDevice*>& devs);
  ~Usb2CanRouter();

  void attach_motor(RobStrideMotor* m);

  void start();
  void stop();

private:
  struct Key {
    int dev_idx;
    uint8_t channel;
    uint8_t motor_id_bits15_8;
    bool operator==(const Key& o) const {
      return dev_idx==o.dev_idx && channel==o.channel && motor_id_bits15_8==o.motor_id_bits15_8;
    }
  };
  struct KeyHash {
    size_t operator()(const Key& k) const noexcept {
      return (size_t(k.dev_idx)&0xFF) | (size_t(k.channel)<<8) | (size_t(k.motor_id_bits15_8)<<16);
    }
  };

  void rx_loop(int dev_idx);

  std::vector<Usb2CanDevice*> devs_;
  std::vector<std::thread> threads_;
  std::atomic<bool> running_{false};

  std::mutex mu_;
  std::unordered_map<Key, RobStrideMotor*, KeyHash> map_;
};
