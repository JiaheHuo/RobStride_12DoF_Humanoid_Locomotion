#pragma once
#include <map>
#include <cmath>
#include <string>
#include <stdexcept>

enum class ActuatorType { RS00, RS02, RS04 };

struct ActuatorOp {
  float pos;   // max position range (+/-)
  float vel;   // max velocity range (+/-)
  float tau;   // max torque range (+/-)
  float kp;    // kp max
  float kd;    // kd max
};

static inline const std::map<ActuatorType, ActuatorOp>& actuator_map() {
  static const std::map<ActuatorType, ActuatorOp> m = {
    {ActuatorType::RS00, {float(4*M_PI), 50.f,  17.f,  500.f,   5.f}},
    {ActuatorType::RS02, {float(4*M_PI), 44.f,  17.f,  500.f,   5.f}},
    {ActuatorType::RS04, {float(4*M_PI), 15.f, 120.f, 5000.f, 100.f}},
  };
  return m;
}

static inline ActuatorType actuator_from_string(const std::string& s) {
  if (s == "RS00") return ActuatorType::RS00;
  if (s == "RS02") return ActuatorType::RS02;
  if (s == "RS04") return ActuatorType::RS04;
  throw std::runtime_error("Unknown actuator type: " + s);
}

static inline std::string actuator_to_string(ActuatorType t) {
  switch (t) {
    case ActuatorType::RS00: return "RS00";
    case ActuatorType::RS02: return "RS02";
    case ActuatorType::RS04: return "RS04";
  }
  return "UNKNOWN";
}
