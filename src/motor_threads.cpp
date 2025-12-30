#include "motor_threads.hpp"

static const char* mode_name(CtrlMode m) {
  switch (m) {
    case CtrlMode::DISABLE: return "DISABLE";
    case CtrlMode::MOTION:  return "MOTION";
    case CtrlMode::HOLD_Q:  return "HOLD_Q";
    default: return "UNK";
  }
}

void StateSyncThread::start() { th_ = std::thread(&StateSyncThread::run_, this); }
void StateSyncThread::join()  { if (th_.joinable()) th_.join(); }

void StateSyncThread::run_() {
  PeriodicTimer timer(RS_STATE_HZ);
  auto* bus = ctx_->bus;
  auto* sys = ctx_->sys;

  while (!ctx_->stop.load()) {
    const uint64_t tns = now_ns();

    for (int i = 0; i < (int)ctx_->motors.size(); ++i) {
      auto* m = ctx_->motors[i];

      MotorFeedback fb;
      auto opt = m->last_pvtt();           // ✅ 你的真实接口
      if (opt.has_value()) {
        const PVTT pv = opt.value();
        fb.q = pv.p;
        fb.dq = pv.v;
        fb.tau = pv.t;
        fb.temp = pv.temp;
        fb.online = true;
      } else {
        fb.online = false;
      }
      fb.stamp_ns = tns;

      bus->set_feedback(i, fb);
    }

    sys->state_sync_count.fetch_add(1);
    sys->last_sync_ns.store(tns);
    timer.sleep();
  }
}

void ControlTxThread::start() { th_ = std::thread(&ControlTxThread::run_, this); }
void ControlTxThread::join()  { if (th_.joinable()) th_.join(); }

void ControlTxThread::run_() {
  PeriodicTimer timer(RS_CONTROL_HZ);
  auto* bus = ctx_->bus;
  auto* sys = ctx_->sys;

  while (!ctx_->stop.load()) {
    const uint64_t tns = now_ns();

    // 急停：对所有电机 stop
    if (sys->estop.load()) {
      for (auto* m : ctx_->motors) {
        m->stop((uint8_t)RS_STOP_CLEAR_ERROR);  // ✅ 你的真实接口
      }
      timer.sleep();
      continue;
    }

    for (int i = 0; i < (int)ctx_->motors.size(); ++i) {
      auto* m = ctx_->motors[i];
      MotorCommand motorCmd = bus->get_command(i);
      MotorFeedback fb = bus->get_feedback(i);

      // 看门狗：命令太久没刷新 -> stop
      uint64_t age_ms = 0;
      if (motorCmd.stamp_ns > 0) age_ms = (tns - motorCmd.stamp_ns) / 1000000ULL;
      if (age_ms > RS_CMD_WATCHDOG_MS) {
        motorCmd.mode = CtrlMode::DISABLE;
        motorCmd.enable = false;
      }

      if (!motorCmd.enable || motorCmd.mode == CtrlMode::DISABLE) {
        m->stop((uint8_t)RS_STOP_CLEAR_ERROR);
        continue;
      }

      if (motorCmd.mode == CtrlMode::HOLD_Q) {
        motorCmd.mode = CtrlMode::MOTION;
        motorCmd.pos = fb.q;
        motorCmd.vel = 0.f;
        motorCmd.torque = 0.f;
        // kp/kd 使用 motorCmd 内的值（默认 build_flags 里）
      }

      // ✅ 你的真实下发接口
      // torque, pos_rad, vel_rad_s, kp, kd
      m->send_motion_command(motorCmd.torque, motorCmd.pos, motorCmd.vel, motorCmd.kp, motorCmd.kd);
    }

    sys->tx_count.fetch_add(1);
    sys->last_tx_ns.store(tns);
    timer.sleep();
  }
}

void TelemetryThread::start() { th_ = std::thread(&TelemetryThread::run_, this); }
void TelemetryThread::join()  { if (th_.joinable()) th_.join(); }

void TelemetryThread::run_() {
#if RS_ENABLE_TELEMETRY
  PeriodicTimer timer(RS_TELEMETRY_HZ);
  auto* bus = ctx_->bus;
  auto* sys = ctx_->sys;

  while (!ctx_->stop.load()) {
    auto snap = bus->snapshot(*sys);

    std::printf("\n==== Telemetry motors=%d sync=%llu tx=%llu estop=%d ====\n",
                (int)snap.fb.size(),
                (unsigned long long)snap.state_sync_count,
                (unsigned long long)snap.tx_count,
                (int)snap.estop);

    for (int i = 0; i < (int)snap.fb.size(); ++i) {
      const auto& f = snap.fb[i];
      const auto& c = snap.motorCmd[i];
      const unsigned long long age =
          (c.stamp_ns == 0) ? 0ULL : (unsigned long long)((now_ns() - c.stamp_ns) / 1000000ULL);

      std::printf(
          "M[%02d] online=%d q=% .4f dq=% .4f tau=% .3f temp=% .1f | motorCmd(%s,en=%d) tq=% .3f p=% .4f v=% .4f kp=%.1f kd=%.1f age=%llums\n",
          i,
          (int)f.online, f.q, f.dq, f.tau, f.temp,
          mode_name(c.mode), (int)c.enable,
          c.torque, c.pos, c.vel, c.kp, c.kd, age
      );
    }

    timer.sleep();
  }
#else
  (void)ctx_;
#endif
}
