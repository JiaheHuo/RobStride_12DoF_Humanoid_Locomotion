#include "config.hpp"
#include "util.hpp"
#include <yaml-cpp/yaml.h>
#include <stdexcept>

static uint8_t parse_u8_any(const YAML::Node& n) {
  if (n.IsScalar()) {
    std::string s = n.as<std::string>();
    return (uint8_t)parse_u32_hex_or_dec(s);
  }
  throw std::runtime_error("parse_u8_any failed");
}

AppConfig load_config_yaml(const std::string& file) {
  YAML::Node root = YAML::LoadFile(file);
  AppConfig cfg;

  if (root["master_id"]) cfg.master_id = parse_u8_any(root["master_id"]);

  if (!root["devices"]) throw std::runtime_error("YAML missing 'devices'");

  for (const auto& devN : root["devices"]) {
    DeviceSpec dev;
    dev.name = devN["name"].as<std::string>();
    dev.path = devN["path"].as<std::string>();

    for (const auto& busN : devN["buses"]) {
      BusSpec bus;
      bus.channel = (uint8_t)busN["channel"].as<int>();
      if (bus.channel != 1 && bus.channel != 2) throw std::runtime_error("channel must be 1 or 2");

      for (const auto& mN : busN["motors"]) {
        MotorSpec ms;
        ms.id = (uint8_t)mN["id"].as<int>();
        ms.type = actuator_from_string(mN["type"].as<std::string>());
        ms.enabled = mN["enabled"] ? mN["enabled"].as<bool>() : true;
        ms.group = mN["group"] ? mN["group"].as<std::string>() : "";
        bus.motors.push_back(ms);
      }
      dev.buses.push_back(bus);
    }
    cfg.devices.push_back(dev);
  }
  return cfg;
}
