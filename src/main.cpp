// #include <filesystem>
// #include <iostream>
// #include <unistd.h>
// #include <limits.h>
// #include <csignal>
// #include <vector>
// #include <memory>
// #include <atomic>
// #include <chrono>
// #include <thread>
// #include <cstdio>
// #include <string>
// #include <cmath>

// #include "usb2can_transport.hpp"
// #include "ankle_kinematics.hpp"
// #include "actuator.hpp"
// #include "databus.hpp"
// #include "config.hpp"
// #include "router.hpp"
// #include "motor.hpp"
// #include "stats.hpp"
// #include "util.hpp"

// static void motorState_update_state_from_motor(RobStrideMotor *m, MotorState &s)
// {
//   auto pvtt_opt = m->last_pvtt();
//   if (!pvtt_opt)
//   {
//     s.online = false;
//     return;
//   }

//   const auto &pvtt = *pvtt_opt;
//   s.online = true;

//   s.q = pvtt.p;
//   s.dq = pvtt.v;
//   s.tau = pvtt.t;
//   s.temp = pvtt.temp;

//   s.age_ms = 0;
// }
// // static void jointState_update_state_from_motor(RobStrideMotor *m, JointState &j)
// // {
// //   auto pvtt_opt = m->last_pvtt();
// //   // m->motor_id
// //   const auto &pvtt = *pvtt_opt;
// //   j.q_joint = pvtt.p;
// //   j.dq_joint = pvtt.v;
// //   j.tau_joint = pvtt.t;
// // }
// static void db_apply_cmd_to_motor(RobStrideMotor *m, const MotorCmd &c)
// {
//   if (!c.enable || c.mode == CtrlMode::DISABLE)
//   {
//     m->send_motion_command(0.f, 0.f, 0.f, 0.f, 0.f);
//     return;
//   }

//   if (c.mode == CtrlMode::MOTION)
//   {
//     m->send_motion_command(c.tau_ff, c.q_des, c.dq_des, c.kp, c.kd);
//     return;
//   }

//   if (c.mode == CtrlMode::TORQUE)
//   {
//     m->send_motion_command(c.tau_ff, 0.f, 0.f, 0.f, 0.f);
//     return;
//   }

//   // fallback
//   m->send_motion_command(0.f, 0.f, 0.f, 0.f, 0.f);
// }

// static std::atomic<bool> g_run{true};
// static void on_sigint(int) { g_run.store(false); }

// struct Args
// {
//   std::string config = "../config/motors.yaml";
//   std::string only_group_csv; // e.g. "u1b1,u2b2"
//   int rate_hz = 500;
//   std::string test = "damping"; // damping / sine / hold
//   float sine_amp = 0.3f;        // rad
//   float sine_freq = 0.5f;       // Hz
//   float kp = 40.f;
//   float kd = 2.0f;
//   bool dry_run = false;
// };

// static Args parse_args(int argc, char **argv)
// {
//   Args a;
//   for (int i = 1; i < argc; i++)
//   {
//     std::string k = argv[i];
//     auto need = [&](const std::string &name)
//     {
//       if (i + 1 >= argc)
//       {
//         std::cerr << "missing value for " << name << "\n";
//         std::exit(1);
//       }
//       return std::string(argv[++i]);
//     };
//     if (k == "--config")
//       a.config = need(k);
//     else if (k == "--only-group")
//       a.only_group_csv = need(k);
//     else if (k == "--rate")
//       a.rate_hz = std::stoi(need(k));
//     else if (k == "--test")
//       a.test = need(k);
//     else if (k == "--amp")
//       a.sine_amp = std::stof(need(k));
//     else if (k == "--freq")
//       a.sine_freq = std::stof(need(k));
//     else if (k == "--kp")
//       a.kp = std::stof(need(k));
//     else if (k == "--kd")
//       a.kd = std::stof(need(k));
//     else if (k == "--dry-run")
//       a.dry_run = true;
//     else if (k == "--help")
//     {
//       std::cout << "Usage:\n"
//                    "  ./motor_test --config config/motors.yaml \n"
//                    "              [--only-group u1b1,u2b2]\n"
//                    "              [--rate 1000]\n"
//                    "              [--test damping|sine|hold]\n"
//                    "              [--amp 0.3 --freq 0.5]\n"
//                    "              [--kp 20 --kd 1]\n"
//                    "              [--dry-run]   (load yaml + select motors, but DO NOT open /dev/USB2CAN*)\n";
//       std::exit(0);
//     }
//   }
//   return a;
// }

// static bool group_selected(const std::string &group, const std::vector<std::string> &only_groups)
// {
//   if (only_groups.empty())
//     return true;
//   for (auto &g : only_groups)
//     if (g == group)
//       return true;
//   return false;
// }

// int main(int argc, char **argv)
// {
//   std::signal(SIGINT, on_sigint);

//   Args args = parse_args(argc, argv);
//   auto only_groups = split_csv(args.only_group_csv);

//   std::cout << "Loading config: " << args.config << "\n";
//   AppConfig cfg = load_config_yaml(args.config);
//   // 创建踝关节
//   ankle::AnkleParams P = ankle::make_params_v2();

//   if (args.dry_run)
//   {
//     Stats stats;

//     struct Sel
//     {
//       MotorKey mk;
//       int sidx;
//     };
//     std::vector<Sel> selected;
//     selected.reserve(64);

//     for (int di = 0; di < (int)cfg.devices.size(); ++di)
//     {
//       auto &devSpec = cfg.devices[di];
//       for (auto &bus : devSpec.buses)
//       {
//         for (auto &ms : bus.motors)
//         {
//           bool sel = ms.enabled && group_selected(ms.group, only_groups);
//           if (!sel)
//             continue;

//           MotorKey mk;
//           mk.dev_idx = di;
//           mk.channel = bus.channel;
//           mk.motor_id = ms.id;
//           mk.dev_name = devSpec.name;
//           mk.group = ms.group;
//           mk.type_str = actuator_to_string(ms.type);
//           mk.motor_name = ms.motorName;
//           int sidx = stats.register_motor(mk);
//           selected.push_back(Sel{mk, sidx});
//         }
//       }
//     }

//     std::cout << "[DRY-RUN] Selected motors: " << selected.size() << "\n";
//     if (selected.empty())
//     {
//       std::cout << "[DRY-RUN] No motors selected. Check YAML enabled or --only-group.\n";
//       return 0;
//     }
//     for (auto &s : selected)
//     {
//       std::cout << "  " << s.mk.motor_name << "  "
//                 << s.mk.dev_name << ":ch" << int(s.mk.channel) << ":id" << int(s.mk.motor_id)
//                 << " type=" << s.mk.type_str << " group=" << s.mk.group << "\n";
//     }

//     int dt_us = int(1e6 / std::max(1, args.rate_hz));
//     auto next = std::chrono::steady_clock::now();
//     auto t0 = std::chrono::steady_clock::now();

//     std::unordered_map<std::string, int> name2dummy;
//     name2dummy.reserve(selected.size());
//     for (int i = 0; i < (int)selected.size(); ++i)
//     {
//       name2dummy[selected[i].mk.motor_name] = i;
//     }

//     auto has_name = [&](const char *n) -> bool
//     {
//       return name2dummy.find(n) != name2dummy.end();
//     };

//     std::cout << "[DRY-RUN] Control loop running (no hardware). Ctrl+C to stop.\n";
//     while (g_run.load())
//     {
//       auto now = std::chrono::steady_clock::now();
//       float t = std::chrono::duration<float>(now - t0).count();

//       // -----------------------------
//       // 新增 FK/IK 自洽测试模式
//       // -----------------------------
//       if (args.test == "fkik")
//       {
//         if (!has_name("L_ankle_L") || !has_name("L_ankle_R"))
//         {
//           static int once = 0;
//           if (!once++)
//           {
//             std::cout << "[DRY-RUN][fkik] Missing motorName L_ankle_L / L_ankle_R in selected set.\n";
//             std::cout << "Selected motorNames:\n";
//             for (auto &s : selected)
//               std::cout << "  " << s.mk.motor_name << "\n";
//           }
//         }

//         // 运动学全用角度制
//         constexpr double RAD2DEG = 180.0 / M_PI;
//         double amp_deg = args.sine_amp * RAD2DEG;
//         double w = 2.0 * M_PI * args.sine_freq;

//         // ========== (1) FK -> IK ==========
//         double m1_deg = amp_deg * std::sin(w * t);
//         double m2_deg = 0.8 * amp_deg * std::cos(w * t);

//         auto fk1 = ankle::forwardKinematics(P, m1_deg, m2_deg, 0.0, 0.0, 80, 1e-10);

//         double e_m1 = NAN, e_m2 = NAN;
//         double pitch_deg = NAN, roll_deg = NAN;

//         if (fk1.ok)
//         {
//           pitch_deg = fk1.joint_deg.pitch_deg;
//           roll_deg = fk1.joint_deg.roll_deg;

//           auto ik1 = ankle::inverseKinematics(P, pitch_deg, roll_deg);
//           double m1_hat = ik1.sol_minus.m1_deg;
//           double m2_hat = ik1.sol_minus.m2_deg;

//           e_m1 = std::abs(m1_hat - m1_deg);
//           e_m2 = std::abs(m2_hat - m2_deg);
//         }

//         // ========== (2) IK -> FK ==========
//         double pitch_des = 0.6 * amp_deg * std::sin(w * t);
//         double roll_des = 0.5 * amp_deg * std::cos(w * t);

//         auto ik2 = ankle::inverseKinematics(P, pitch_des, roll_des);
//         double m1_from_joint = ik2.sol_minus.m1_deg;
//         double m2_from_joint = ik2.sol_minus.m2_deg;

//         auto fk2 = ankle::forwardKinematics(P, m1_from_joint, m2_from_joint, 0.0, 0.0, 80, 1e-10);

//         double e_pitch = NAN, e_roll = NAN;
//         if (fk2.ok)
//         {
//           e_pitch = std::abs(fk2.joint_deg.pitch_deg - pitch_des);
//           e_roll = std::abs(fk2.joint_deg.roll_deg - roll_des);
//         }

//         // 打印 2Hz
//         static int print_div = 0;
//         if (++print_div >= std::max(1, args.rate_hz / 2))
//         {
//           print_div = 0;
//           std::cout
//               << "[DRY-RUN][fkik] FK->IK  m=(" << m1_deg << "," << m2_deg << ")deg"
//               << " joint=(" << pitch_deg << "," << roll_deg << ")deg"
//               << " err_m=(" << e_m1 << "," << e_m2 << ")deg"
//               << " | IK->FK joint_des=(" << pitch_des << "," << roll_des << ")deg"
//               << " err_joint=(" << e_pitch << "," << e_roll << ")deg"
//               << "\n";
//         }
//       }
//       else
//       {
//         for (auto &s : selected)
//         {
//           (void)s;
//           float torque = 0.f, pos = 0.f, vel = 0.f;

//           if (args.test == "damping")
//           {
//             pos = 0.f;
//             vel = 0.f;
//             (void)torque;
//           }
//           else if (args.test == "hold")
//           {
//             (void)torque;
//           }
//           else if (args.test == "sine")
//           {
//             float w = 2.0f * float(M_PI) * args.sine_freq;
//             pos = args.sine_amp * std::sin(w * t);
//             vel = args.sine_amp * w * std::cos(w * t);
//             (void)pos;
//             (void)vel;
//           }
//         }
//       }

//       stats.print_periodic();
//       next += std::chrono::microseconds(dt_us);
//       std::this_thread::sleep_until(next);
//     }

//     std::cout << "[DRY-RUN] Stopped.\n";
//     return 0;
//   }
//   std::vector<bool> need_dev(cfg.devices.size(), false);

//   for (int di = 0; di < (int)cfg.devices.size(); ++di)
//   {
//     auto &devSpec = cfg.devices[di];
//     for (auto &bus : devSpec.buses)
//     {
//       for (auto &ms : bus.motors)
//       {
//         bool sel = ms.enabled && group_selected(ms.group, only_groups);
//         if (sel)
//         {
//           need_dev[di] = true;
//           break;
//         }
//       }
//       if (need_dev[di])
//         break;
//     }
//   }

//   // 2) 只打开需要的设备（其他不碰，避免 openfail USB2CAN0）
//   std::vector<std::unique_ptr<Usb2CanDevice>> dev_objs;
//   std::vector<Usb2CanDevice *> dev_ptrs(cfg.devices.size(), nullptr); // 注意：按 cfg 索引对齐

//   dev_objs.reserve(cfg.devices.size());

//   for (int di = 0; di < (int)cfg.devices.size(); ++di)
//   {
//     auto &d = cfg.devices[di];
//     if (!need_dev[di])
//     {
//       std::cout << "Skip open " << d.name << " (not selected)\n";
//       continue;
//     }
//     auto dev = std::make_unique<Usb2CanDevice>(d.path);
//     std::cout << "Opened " << d.name << " at " << d.path << "\n";
//     dev_ptrs[di] = dev.get();
//     dev_objs.push_back(std::move(dev));
//   }

//   // 2) 构建 motors（支持 only-group + enabled）
//   Stats stats;
//   std::vector<std::unique_ptr<RobStrideMotor>> motors;
//   std::vector<RobStrideMotor *> active;
//   // 加入kpkd
//   std::vector<float> kp_by_idx;
//   std::vector<float> kd_by_idx;
//   kp_by_idx.reserve(64);
//   kd_by_idx.reserve(64);

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

//         BusHandle bh;
//         bh.dev_idx = di;
//         bh.dev_name = devSpec.name;
//         bh.dev = dev_ptrs[di];
//         bh.channel = bus.channel;

//         MotorKey mk;
//         mk.dev_idx = di;
//         mk.channel = bus.channel;
//         mk.motor_id = ms.id;
//         mk.dev_name = devSpec.name;
//         mk.group = ms.group;
//         mk.type_str = actuator_to_string(ms.type);
//         mk.motor_name = ms.motorName;

//         int sidx = stats.register_motor(mk);

//         motors.push_back(std::make_unique<RobStrideMotor>(
//             bh, cfg.master_id, ms.id, ms.type, ms.group, &stats, sidx, ms.motorName));
//         kp_by_idx.push_back(ms.kp);
//         kd_by_idx.push_back(ms.kd);
//       }
//     }
//   }

//   for (auto &m : motors)
//     active.push_back(m.get());

//   std::cout << "Active motors: " << active.size() << "\n";
//   if (active.empty())
//   {
//     std::cout << "No motors selected. Check YAML enabled or --only-group.\n";
//     return 0;
//   }

//   // 3) 启动 router
//   std::vector<Usb2CanDevice *> opened_devs;
//   opened_devs.reserve(cfg.devices.size());
//   for (auto *p : dev_ptrs)
//     if (p)
//       opened_devs.push_back(p);

//   Usb2CanRouter router(opened_devs);

//   for (auto *m : active)
//     router.attach_motor(m);
//   router.start();

//   // 从motorName找Idx
//   std::unordered_map<std::string, int> idx;
//   idx.reserve(active.size());
//   for (int i = 0; i < (int)active.size(); ++i)
//   {
//     idx[active[i]->motorName_] = i;
//   }

//   // ===== logging config =====
//   int log_every = 5; // 每隔 N 次循环记录一次（1000Hz 时 N=2 => 500Hz）
//   const std::string log_dir = "/home/jhuo/robstride_usb2can_ctrl/logs";
//   const std::string log_path = log_dir + "/track.csv";

//   // 确保目录存在
//   try
//   {
//     std::filesystem::create_directories(log_dir);
//   }
//   catch (const std::exception &e)
//   {
//     std::cerr << "[log] create_directories failed: " << e.what() << "\n";
//     return 1;
//   }

//   std::cout << "[log] track.csv => " << log_path << "\n";

//   FILE *flog = std::fopen(log_path.c_str(), "w");
//   if (!flog)
//   {
//     std::perror("[log] open log file failed");
//     return 1;
//   }

//   // 行缓冲：每行带 '\n' 就会更及时地落盘（即使你没频繁 fflush）
//   setvbuf(flog, nullptr, _IOLBF, 0);

//   // CSV header
//   std::fprintf(flog, "t,i,q,dq,q_des,dq_des\n");
//   std::fflush(flog);

//   // 创建 DataBus
//   DataBus databus((int)active.size());
//   databus.disable_all();
//   // 4) 上电序列：stop -> set_mode(0) -> enable
//   std::cout << "Init motors: stop -> set_mode(0) -> enable -> set_zero\n";
//   for (auto *m : active)
//   {
//     m->stop(0);
//     std::this_thread::sleep_for(std::chrono::milliseconds(2));
//     m->set_mode(0); // 运控
//     std::this_thread::sleep_for(std::chrono::milliseconds(2));
//     m->enable();
//     std::this_thread::sleep_for(std::chrono::milliseconds(2));
//     for (int k = 0; k < 10; ++k)
//     {
//       m->send_motion_command(0.0f, 0.0f, 0.0f, 0.0f, args.kd);
//       std::this_thread::sleep_for(std::chrono::milliseconds(1));
//     }
//     m->set_zero();
//     std::this_thread::sleep_for(std::chrono::milliseconds(10));
//   }

//   // 5) 控制循环
//   int dt_us = int(1e6 / std::max(1, args.rate_hz));
//   auto next = std::chrono::steady_clock::now();
//   auto t0 = std::chrono::steady_clock::now();

//   std::cout << "Control loop running. Ctrl+C to stop.\n";
//   while (g_run.load())
//   {
//     auto now = std::chrono::steady_clock::now();
//     float t = std::chrono::duration<float>(now - t0).count();
//     // 写数据
//     Eigen::VectorXd motorPos_eigen = Eigen::VectorXd::Zero((int)active.size());

//     for (size_t i = 0; i < active.size(); ++i)
//     {
//       motorState_update_state_from_motor(active[i], databus.motorState((int)i));
//       // jointState_update_state_from_motor(active[i], databus.jointState((int)i));
//       motorPos_eigen((int)i) = databus.motorState((int)i).q;
//     }

//     constexpr double RAD2DEG = 180.0 / M_PI;
//     constexpr double DEG2RAD = M_PI / 180.0;
//     double m1_deg = motorPos_eigen(idx.at("L_ankle_L")) * RAD2DEG;
//     double m2_deg = motorPos_eigen(idx.at("L_ankle_R")) * RAD2DEG;

//     auto fk = ankle::forwardKinematics(P, m1_deg, m2_deg, 0.0, 0.0, 80, 1e-10);
//     Eigen::VectorXd jointPos_eigen = Eigen::VectorXd::Zero((int)active.size());
//     jointPos_eigen = motorPos_eigen.eval();
//     jointPos_eigen(idx.at("L_ankle_L")) = fk.joint_deg.pitch_deg * DEG2RAD;
//     jointPos_eigen(idx.at("L_ankle_R")) = fk.joint_deg.roll_deg * DEG2RAD;

//     // for (auto *m : active)
//     // {
//     //   float torque = 0.f;
//     //   float pos = 0.f;
//     //   float vel = 0.f;
//     //   if (args.test == "damping") {
//     //     // 纯阻尼：kp=0, kd>0
//     //     pos = 0.f; vel = 0.f;
//     //     m->send_motion_command(torque, pos, vel, 0.0f, args.kd);
//     //   } else if (args.test == "hold") {
//     //     // 固定位置：pos=0
//     //     m->send_motion_command(torque, 0.0f, 0.0f, args.kp, args.kd);
//     //   } else if (args.test == "sine") {
//     //     float w = 2.0f * float(M_PI) * args.sine_freq;
//     //     pos = args.sine_amp * std::sin(w * t);
//     //     vel = args.sine_amp * w * std::cos(w * t);
//     //     m->send_motion_command(torque, pos, vel, args.kp, args.kd);
//     //   } else {
//     //     // 默认：不动
//     //     m->send_motion_command(0.f, 0.f, 0.f, 0.f, 0.f);
//     //   }
//     // }
//     for (size_t i = 0; i < active.size(); ++i)
//     {
//       auto &motorCmd = databus.motorCmd((int)i);

//       if (args.test == "damping")
//       {
//         // 纯阻尼：kp=0, kd>0
//         motorCmd.enable = true;
//         motorCmd.mode = CtrlMode::MOTION;
//         motorCmd.tau_ff = 0.f;
//         motorCmd.q_des = 0.f;
//         motorCmd.dq_des = 0.f;
//         motorCmd.kp = 0.f;
//         motorCmd.kd = kd_by_idx[i]; // YAML
//       }
//       else if (args.test == "hold")
//       {
//         motorCmd.enable = true;
//         motorCmd.mode = CtrlMode::MOTION;
//         motorCmd.tau_ff = 0.f;
//         motorCmd.q_des = 0.f;
//         motorCmd.dq_des = 0.f;
//         // motorCmd.kp     = args.kp;
//         motorCmd.kp = kp_by_idx[i];
//         motorCmd.kd = kd_by_idx[i];
//       }
//       else if (args.test == "sine")
//       {
//         if (!databus.sine((int)i).armed && databus.motorState((int)i).online)
//         {
//           databus.sine((int)i).amp_rad = args.sine_amp;
//           databus.sine((int)i).freq_hz = args.sine_freq;
//           databus.sine((int)i).kp = args.kp;
//           databus.sine((int)i).kd = args.kd;
//           // 用yaml覆盖！
//           databus.sine((int)i).kp = kp_by_idx[i];
//           databus.sine((int)i).kd = kd_by_idx[i];
//           databus.arm_sine_zero((int)i); // q0 = current q
//         }

//         // 生成 sine 指令写入 cmd（内部写 motorCmd.enable/mode/q_des/kp/kd）
//         databus.write_sine_cmd((int)i, (double)t);
//       }
//       else
//       {
//         // 默认：disable
//         motorCmd = MotorCmd{};
//       }
//     }
//     // std::cout << "Active gains (YAML order):\n";
//     // for (int i = 0; i < (int)active.size(); ++i)
//     // {
//     //   std::cout << "  idx " << i << " kp=" << kp_by_idx[i] << " kd=" << kd_by_idx[i] << "\n";
//     // }

//     // 3) 从 databus.motorCmd 读出 -> 下发到电机 在这之前存log

//     static int log_div = 0;
//     if (++log_div >= log_every)
//     {
//       log_div = 0;

//       // 一次写 active.size() 行（比如 6 行）
//       for (int i = 0; i < (int)active.size(); ++i)
//       {
//         const auto &s = databus.motorState(i);
//         const auto &c = databus.motorCmd(i);

//         std::fprintf(flog, "%.6f,%d,%.6f,%.6f,%.6f,%.6f\n",
//                      (double)t, i,
//                      (double)s.q, (double)s.dq,
//                      (double)c.q_des, (double)c.dq_des);
//       }

//       // flush：按“批次”计数，不要每行 flush
//       static int flush_batches = 0;
//       if (++flush_batches >= 100)
//       { // 100个批次（每批6行）刷一次
//         flush_batches = 0;
//         std::fflush(flog);
//       }
//     }
//     Eigen::VectorXd motorPosCmd_eigen = Eigen::VectorXd::Zero((int)active.size());
//     for (size_t i = 0; i < active.size(); ++i)
//     {
//       db_apply_cmd_to_motor(active[i], databus.motorCmd((int)i));
//       motorPosCmd_eigen((int)i) = databus.motorCmd((int)i).q_des;
//     }
//     double left_pitch_target = motorPosCmd_eigen(idx.at("L_ankle_L")) * RAD2DEG;
//     double left_roll_target = motorPosCmd_eigen(idx.at("L_ankle_R")) * RAD2DEG;

//     auto ik = ankle::inverseKinematics(P, left_pitch_target, left_roll_target);
//     Eigen::VectorXd jointPosCmd_eigen = motorPosCmd_eigen.eval();
//     jointPosCmd_eigen(idx.at("L_ankle_L")) = ik.sol_minus.m1_deg * DEG2RAD;
//     jointPosCmd_eigen(idx.at("L_ankle_R")) = ik.sol_minus.m2_deg * DEG2RAD;
//     // 4) 打印
//     static int print_div = 0;
//     if (++print_div >= std::max(1, args.rate_hz / 2))
//     { // 2Hz
//       print_div = 0;
//       databus.print_telemetry_line("Telemetry(databus)");
//     }

//     // 每秒打印统计
//     stats.print_periodic();

//     next += std::chrono::microseconds(dt_us);
//     std::this_thread::sleep_until(next);
//   }

//   std::cout << "Stopping...\n";
//   for (auto *m : active)
//   {
//     m->stop(0);
//     std::this_thread::sleep_for(std::chrono::milliseconds(1));
//   }
//   // 关闭log
//   if (flog)
//   {
//     std::fflush(flog);
//     std::fclose(flog);
//     flog = nullptr;
//   }

//   router.stop();
//   return 0;
// }
#include <csignal>
#include <atomic>

#include "GridRobot.hpp"

static std::atomic<bool> g_run{true};
static void on_sigint(int){ g_run.store(false); }

int main(int argc, char** argv) {
  std::signal(SIGINT, on_sigint);

  auto args = GridRobot::parse_args(argc, argv);

  GridRobot robot;
  if (!robot.init(args)) return 0;   // init 内已打印原因
  robot.exec(g_run);
  return 0;
}
