#include "stats.hpp"
#include <iostream>
#include <iomanip>
#include <map>

int Stats::register_motor(const MotorKey& k) {
  motors_.emplace_back();        // 原地构造，不触发 move
  motors_.back().key = k;
  return (int)motors_.size() - 1;
}

void Stats::inc_tx_motion(int idx){ motors_[idx].cnt.tx_motion.fetch_add(1, std::memory_order_relaxed); }
void Stats::inc_tx_other (int idx){ motors_[idx].cnt.tx_other .fetch_add(1, std::memory_order_relaxed); }
void Stats::inc_rx_status(int idx){ motors_[idx].cnt.rx_status.fetch_add(1, std::memory_order_relaxed); }
void Stats::inc_rx_other (int idx){ motors_[idx].cnt.rx_other .fetch_add(1, std::memory_order_relaxed); }

void Stats::print_periodic() {
  auto now = std::chrono::steady_clock::now();
  double dt = std::chrono::duration<double>(now - last_print_).count();
  if (dt < 1.0) return;
  last_print_ = now;

  // 汇总（按 dev / bus）
  struct Agg { uint64_t tx=0, rx=0, last_tx=0, last_rx=0; };
  std::map<std::string, Agg> devAgg;
  std::map<std::string, Agg> busAgg;

  uint64_t tx_total = 0, rx_total = 0;
  // uint64_t tx_total_prev = 0, rx_total_prev = 0;

  for (auto& e : motors_) {
    uint64_t txm = e.cnt.tx_motion.load(std::memory_order_relaxed);
    uint64_t rxm = e.cnt.rx_status.load(std::memory_order_relaxed);

    uint64_t dtx = txm - e.last_tx_motion;
    uint64_t drx = rxm - e.last_rx_status;

    e.last_tx_motion = txm;
    e.last_rx_status = rxm;

    tx_total += dtx;
    rx_total += drx;

    // device key
    devAgg[e.key.dev_name].tx += dtx;
    devAgg[e.key.dev_name].rx += drx;

    // bus key
    std::string bkey = e.key.dev_name + ":bus" + std::to_string((int)e.key.channel);
    busAgg[bkey].tx += dtx;
    busAgg[bkey].rx += drx;
  }

  double tx_hz = tx_total / dt;
  double rx_hz = rx_total / dt;
  double drop_est = (tx_total > 0) ? (1.0 - double(rx_total)/double(tx_total)) : 0.0;

  std::cout << std::fixed << std::setprecision(2);
  std::cout << "\n[" << dt << "s] "
            << "TX_motion=" << tx_total << " (" << tx_hz << " Hz)  "
            << "RX_status=" << rx_total << " (" << rx_hz << " Hz)  "
            << "drop_est~=" << (drop_est*100.0) << "%\n";

  for (auto& [k, a] : devAgg) {
    double d = (a.tx>0) ? (1.0 - double(a.rx)/double(a.tx)) : 0.0;
    std::cout << "  " << k << ": TX=" << a.tx << " RX=" << a.rx
              << " drop~=" << (d*100.0) << "%\n";
  }
  for (auto& [k, a] : busAgg) {
    double d = (a.tx>0) ? (1.0 - double(a.rx)/double(a.tx)) : 0.0;
    std::cout << "    " << k << ": TX=" << a.tx << " RX=" << a.rx
              << " drop~=" << (d*100.0) << "%\n";
  }

  std::cout << "  (drop_est 是工程估计：假设每发一次运控帧应回一次状态帧；若你的电机不是这种机制，需要改成按期望反馈频率统计)\n";
}
