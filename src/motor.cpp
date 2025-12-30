#include "motor.hpp"
#include <cstring>

RobStrideMotor::RobStrideMotor(BusHandle bus, uint8_t master_id, uint8_t motor_id, ActuatorType type,
                               std::string group, Stats* stats, int stats_idx)
: bus_(bus), master_id_(master_id), motor_id_(motor_id), type_(type),
  group_(std::move(group)), stats_(stats), stats_idx_(stats_idx) {}

void RobStrideMotor::enable() {
  std::array<uint8_t,8> data{}; data.fill(0);
  uint32_t eid = rs::make_eid((uint8_t)rs::Comm::MotorEnable, master_id_, motor_id_);
  bus_.dev->send(bus_.channel, eid, data, 8);
  if (stats_) stats_->inc_tx_other(stats_idx_);
}

void RobStrideMotor::stop(uint8_t clear_error) {
  std::array<uint8_t,8> data{}; data.fill(0);
  data[0] = clear_error;
  uint32_t eid = rs::make_eid((uint8_t)rs::Comm::MotorStop, master_id_, motor_id_);
  bus_.dev->send(bus_.channel, eid, data, 8);
  if (stats_) stats_->inc_tx_other(stats_idx_);
}

void RobStrideMotor::set_zero() {
  std::array<uint8_t,8> data{}; data.fill(0);
  data[0] = 1;
  uint32_t eid = rs::make_eid((uint8_t)rs::Comm::SetPosZero, master_id_, motor_id_);
  bus_.dev->send(bus_.channel, eid, data, 8);
  if (stats_) stats_->inc_tx_other(stats_idx_);
}

void RobStrideMotor::set_param_f32(uint16_t index, float value) {
  std::array<uint8_t,8> data{}; data.fill(0);
  data[0] = (uint8_t)(index & 0xFF);
  data[1] = (uint8_t)(index >> 8);
  std::memcpy(&data[4], &value, 4);

  uint32_t eid = rs::make_eid((uint8_t)rs::Comm::SetParam, master_id_, motor_id_);
  bus_.dev->send(bus_.channel, eid, data, 8);
  if (stats_) stats_->inc_tx_other(stats_idx_);
}

void RobStrideMotor::set_param_u8(uint16_t index, uint8_t value) {
  std::array<uint8_t,8> data{}; data.fill(0);
  data[0] = (uint8_t)(index & 0xFF);
  data[1] = (uint8_t)(index >> 8);
  data[4] = value;

  uint32_t eid = rs::make_eid((uint8_t)rs::Comm::SetParam, master_id_, motor_id_);
  bus_.dev->send(bus_.channel, eid, data, 8);
  if (stats_) stats_->inc_tx_other(stats_idx_);
}

void RobStrideMotor::get_param(uint16_t index) {
  std::array<uint8_t,8> data{}; data.fill(0);
  data[0] = (uint8_t)(index & 0xFF);
  data[1] = (uint8_t)(index >> 8);

  uint32_t eid = rs::make_eid((uint8_t)rs::Comm::GetParam, master_id_, motor_id_);
  bus_.dev->send(bus_.channel, eid, data, 8);
  if (stats_) stats_->inc_tx_other(stats_idx_);
}

void RobStrideMotor::set_mode(uint8_t run_mode) {
  // 0x7005: run_mode
  set_param_u8(0x7005, run_mode);
}

void RobStrideMotor::send_motion_command(float torque, float pos_rad, float vel_rad_s, float kp, float kd) {
  const auto op = actuator_map().at(type_);

  uint16_t t_u = rs::float_to_u16(torque, -op.tau, op.tau);
  uint32_t eid = rs::make_eid((uint8_t)rs::Comm::MotionControl, t_u, motor_id_);

  std::array<uint8_t,8> data{}; data.fill(0);
  uint16_t p_u  = rs::float_to_u16(pos_rad, -op.pos, op.pos);
  uint16_t v_u  = rs::float_to_u16(vel_rad_s, -op.vel, op.vel);
  uint16_t kp_u = rs::float_to_u16(kp, 0.f, op.kp);
  uint16_t kd_u = rs::float_to_u16(kd, 0.f, op.kd);

  rs::be16_store(data, 0, p_u);
  rs::be16_store(data, 2, v_u);
  rs::be16_store(data, 4, kp_u);
  rs::be16_store(data, 6, kd_u);

  bus_.dev->send(bus_.channel, eid, data, 8);
  if (stats_) stats_->inc_tx_motion(stats_idx_);
}

void RobStrideMotor::on_frame(const Usb2CanFrame& f) {
  auto meta = rs::decode_eid(f.eid);

  if (meta.comm_type == (uint8_t)rs::Comm::MotorRequest) {
    uint16_t pu = (f.data[0] << 8) | f.data[1];
    uint16_t vu = (f.data[2] << 8) | f.data[3];
    uint16_t tu = (f.data[4] << 8) | f.data[5];
    uint16_t te = (f.data[6] << 8) | f.data[7];

    const auto op = actuator_map().at(type_);

    PVTT pv;
    pv.p    = rs::pvtt_decode_u16(pu, op.pos);
    pv.v    = rs::pvtt_decode_u16(vu, op.vel);
    pv.t    = rs::pvtt_decode_u16(tu, op.tau);
    pv.temp = float(te) * 0.1f;

    {
      std::lock_guard<std::mutex> lk(mu_);
      pvtt_ = pv;
      has_pvtt_ = true;
    }
    if (stats_) stats_->inc_rx_status(stats_idx_);
  } else {
    if (stats_) stats_->inc_rx_other(stats_idx_);
  }
}

std::optional<PVTT> RobStrideMotor::last_pvtt() const {
  std::lock_guard<std::mutex> lk(mu_);
  if (!has_pvtt_) return std::nullopt;
  return pvtt_;
}
