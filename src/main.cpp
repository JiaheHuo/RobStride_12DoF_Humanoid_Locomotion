#include <iostream>
#include <vector>
#include <memory>
#include <atomic>
#include <csignal>
#include <chrono>
#include <thread>
#include <cmath>

#include "config.hpp"
#include "usb2can_transport.hpp"
#include "router.hpp"
#include "motor.hpp"
#include "stats.hpp"
#include "util.hpp"
#include "actuator.hpp"

static std::atomic<bool> g_run{true};
static void on_sigint(int){ g_run.store(false); }

struct Args {
  std::string config = "config/motors.yaml";
  std::string only_group_csv;   // e.g. "u1b1,u2b2"
  int rate_hz = 1000;
  std::string test = "damping"; // damping / sine / hold
  float sine_amp = 0.3f;        // rad
  float sine_freq = 0.5f;       // Hz
  float kp = 20.f;
  float kd = 1.0f;
  bool dry_run = false;
};

static Args parse_args(int argc, char** argv) {
  Args a;
  for (int i=1;i<argc;i++) {
    std::string k = argv[i];
    auto need = [&](const std::string& name){
      if (i+1>=argc) { std::cerr<<"missing value for "<<name<<"\n"; std::exit(1); }
      return std::string(argv[++i]);
    };
    if (k=="--config") a.config = need(k);
    else if (k=="--only-group") a.only_group_csv = need(k);
    else if (k=="--rate") a.rate_hz = std::stoi(need(k));
    else if (k=="--test") a.test = need(k);
    else if (k=="--amp") a.sine_amp = std::stof(need(k));
    else if (k=="--freq") a.sine_freq = std::stof(need(k));
    else if (k=="--kp") a.kp = std::stof(need(k));
    else if (k=="--kd") a.kd = std::stof(need(k));
    else if (k=="--dry-run") a.dry_run = true; 
    else if (k=="--help") {
      std::cout <<
        "Usage:\n"
        "  ./motor_test --config config/motors.yaml \n"
        "              [--only-group u1b1,u2b2]\n"
        "              [--rate 1000]\n"
        "              [--test damping|sine|hold]\n"
        "              [--amp 0.3 --freq 0.5]\n"
        "              [--kp 20 --kd 1]\n"
        "              [--dry-run]   (load yaml + select motors, but DO NOT open /dev/USB2CAN*)\n";
      std::exit(0);
    }
  }
  return a;
}

static bool group_selected(const std::string& group, const std::vector<std::string>& only_groups) {
  if (only_groups.empty()) return true;
  for (auto& g : only_groups) if (g == group) return true;
  return false;
}

int main(int argc, char** argv) {
  std::signal(SIGINT, on_sigint);

  Args args = parse_args(argc, argv);
  auto only_groups = split_csv(args.only_group_csv);

  std::cout << "Loading config: " << args.config << "\n";
  AppConfig cfg = load_config_yaml(args.config);
  // ==========================================================
  // DRY-RUN：DO NOT OPEN DEV BUT RUN STATS AND LOAD YAML FILES
  // ==========================================================
  // if (args.dry_run)
  // {
  //   struct Pick
  //   {
  //     std::string dev;
  //     int ch;
  //     int id;
  //     std::string type;
  //     std::string group;
  //   };
  //   std::vector<Pick> picks;

  //   for (int di = 0; di < (int)cfg.devices.size(); ++di)
  //   {
  //     auto &devSpec = cfg.devices[di];
  //     for (auto &bus : devSpec.buses)
  //     {
  //       for (auto &ms : bus.motors)
  //       {
  //         bool sel = ms.enabled && group_selected(ms.group, only_groups);
  //         if (!sel)
  //           continue;

  //         Pick p;
  //         p.dev = devSpec.name;
  //         p.ch = bus.channel;
  //         p.id = ms.id;
  //         p.type = actuator_to_string(ms.type);
  //         p.group = ms.group;
  //         picks.push_back(std::move(p));
  //       }
  //     }
  //   }

  //   std::cout << "[DRY-RUN] Selected motors: " << picks.size() << "\n";
  //   if (picks.empty())
  //   {
  //     std::cout << "[DRY-RUN] No motors selected. Check YAML enabled or --only-group.\n";
  //     return 0;
  //   }

  //   for (auto &p : picks)
  //   {
  //     std::cout << "  " << p.dev << ":ch" << p.ch << ":id" << p.id
  //               << " type=" << p.type
  //               << " group=" << p.group << "\n";
  //   }

  //   std::cout << "[DRY-RUN] Skip opening /dev/USB2CAN*, skip router, skip control loop.\n";
  //   return 0;
  // }
    if (args.dry_run) {
    Stats stats;

    struct Sel {
      MotorKey mk;
      int sidx;
    };
    std::vector<Sel> selected;
    selected.reserve(64);

    // 只做“选择电机 + 注册 stats”
    for (int di=0; di<(int)cfg.devices.size(); ++di) {
      auto& devSpec = cfg.devices[di];
      for (auto& bus : devSpec.buses) {
        for (auto& ms : bus.motors) {
          bool sel = ms.enabled && group_selected(ms.group, only_groups);
          if (!sel) continue;

          MotorKey mk;
          mk.dev_idx  = di;
          mk.channel  = bus.channel;
          mk.motor_id = ms.id;
          mk.dev_name = devSpec.name;
          mk.group    = ms.group;
          mk.type_str = actuator_to_string(ms.type);

          int sidx = stats.register_motor(mk);
          selected.push_back(Sel{mk, sidx});
        }
      }
    }

    std::cout << "[DRY-RUN] Selected motors: " << selected.size() << "\n";
    if (selected.empty()) {
      std::cout << "[DRY-RUN] No motors selected. Check YAML enabled or --only-group.\n";
      return 0;
    }
    for (auto& s : selected) {
      std::cout << "  " << s.mk.dev_name << ":ch" << s.mk.channel << ":id" << s.mk.motor_id
                << " type=" << s.mk.type_str
                << " group=" << s.mk.group << "\n";
    }

    int dt_us = int(1e6 / std::max(1, args.rate_hz));
    auto next = std::chrono::steady_clock::now();
    auto t0 = std::chrono::steady_clock::now();

    std::cout << "[DRY-RUN] Control loop running (no hardware). Ctrl+C to stop.\n";

    while (g_run.load()) {
      auto now = std::chrono::steady_clock::now();
      float t = std::chrono::duration<float>(now - t0).count();

      // 这里“照样执行 test 计算”，但不调用 m->send_motion_command
      // 目的：让你能测主循环负载、stats 打印节奏、参数解析等
      for (auto& s : selected) {
        (void)s; // 当前不发 CAN，所以仅保留计算结构
        float torque = 0.f;
        float pos = 0.f;
        float vel = 0.f;

        if (args.test == "damping") {
          pos = 0.f; vel = 0.f;
          // would send (0, pos, vel, 0, kd)
          (void)torque;
        } else if (args.test == "hold") {
          // would send (0, 0, 0, kp, kd)
          (void)torque;
        } else if (args.test == "sine") {
          float w = 2.0f * float(M_PI) * args.sine_freq;
          pos = args.sine_amp * std::sin(w * t);
          vel = args.sine_amp * w * std::cos(w * t);
          (void)pos; (void)vel;
        } else {
          // default noop
        }
      }

      // 统计照样打印（tx/rx 会是 0，这就是 dry-run 的意义）
      stats.print_periodic();

      next += std::chrono::microseconds(dt_us);
      std::this_thread::sleep_until(next);
    }

    std::cout << "[DRY-RUN] Stopped.\n";
    return 0;
  }
  // 1) 打开三个 USB2CAN
  std::vector<std::unique_ptr<Usb2CanDevice>> dev_objs;
  std::vector<Usb2CanDevice*> dev_ptrs;

  dev_objs.reserve(cfg.devices.size());
  dev_ptrs.reserve(cfg.devices.size());

  for (auto& d : cfg.devices) {
    auto dev = std::make_unique<Usb2CanDevice>(d.path);
    std::cout << "Opened " << d.name << " at " << d.path << "\n";
    dev_ptrs.push_back(dev.get());
    dev_objs.push_back(std::move(dev));
  }

  // 2) 构建 motors（支持 only-group + enabled）
  Stats stats;
  std::vector<std::unique_ptr<RobStrideMotor>> motors;
  std::vector<RobStrideMotor*> active;

  for (int di=0; di<(int)cfg.devices.size(); ++di) {
    auto& devSpec = cfg.devices[di];
    for (auto& bus : devSpec.buses) {
      for (auto& ms : bus.motors) {

        bool sel = ms.enabled && group_selected(ms.group, only_groups);
        if (!sel) continue;

        BusHandle bh;
        bh.dev_idx = di;
        bh.dev_name = devSpec.name;
        bh.dev = dev_ptrs[di];
        bh.channel = bus.channel;

        MotorKey mk;
        mk.dev_idx = di;
        mk.channel = bus.channel;
        mk.motor_id = ms.id;
        mk.dev_name = devSpec.name;
        mk.group = ms.group;
        mk.type_str = actuator_to_string(ms.type);

        int sidx = stats.register_motor(mk);

        motors.push_back(std::make_unique<RobStrideMotor>(
          bh, cfg.master_id, ms.id, ms.type, ms.group, &stats, sidx
        ));
      }
    }
  }

  for (auto& m : motors) active.push_back(m.get());

  std::cout << "Active motors: " << active.size() << "\n";
  if (active.empty()) {
    std::cout << "No motors selected. Check YAML enabled or --only-group.\n";
    return 0;
  }

  // 3) 启动 router
  Usb2CanRouter router(dev_ptrs);
  for (auto* m : active) router.attach_motor(m);
  router.start();

  // 4) 上电序列：stop -> set_mode(0) -> enable
  std::cout << "Init motors: stop -> set_mode(0) -> enable\n";
  for (auto* m : active) {
    m->stop(0);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    m->set_mode(0); // 运控
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    m->enable();
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }

  // 5) 控制循环
  int dt_us = int(1e6 / std::max(1, args.rate_hz));
  auto next = std::chrono::steady_clock::now();
  auto t0 = std::chrono::steady_clock::now();

  std::cout << "Control loop running. Ctrl+C to stop.\n";
  while (g_run.load()) {
    auto now = std::chrono::steady_clock::now();
    float t = std::chrono::duration<float>(now - t0).count();

    for (auto* m : active) {
      float torque = 0.f;
      float pos = 0.f;
      float vel = 0.f;

      if (args.test == "damping") {
        // 纯阻尼：kp=0, kd>0
        pos = 0.f; vel = 0.f;
        m->send_motion_command(torque, pos, vel, 0.0f, args.kd);
      } else if (args.test == "hold") {
        // 固定位置：pos=0
        m->send_motion_command(torque, 0.0f, 0.0f, args.kp, args.kd);
      } else if (args.test == "sine") {
        float w = 2.0f * float(M_PI) * args.sine_freq;
        pos = args.sine_amp * std::sin(w * t);
        vel = args.sine_amp * w * std::cos(w * t);
        m->send_motion_command(torque, pos, vel, args.kp, args.kd);
      } else {
        // 默认：不动
        m->send_motion_command(0.f, 0.f, 0.f, 0.f, 0.f);
      }
    }

    // 每秒打印统计
    stats.print_periodic();

    next += std::chrono::microseconds(dt_us);
    std::this_thread::sleep_until(next);
  }

  std::cout << "Stopping...\n";
  for (auto* m : active) {
    m->stop(0);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  router.stop();
  return 0;
}
