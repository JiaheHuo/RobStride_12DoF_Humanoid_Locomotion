#pragma once
#include <Eigen/Dense>
#include <string>

namespace ankle {

using Eigen::Vector3d;
using Eigen::Matrix2d;
using Eigen::Matrix3d;
using Eigen::Vector2d;

struct FwdOut2D {
  double f1 = 0.0;
  double f2 = 0.0;
  Matrix2d dF = Matrix2d::Zero(); // df/d(alpha,beta)
};

struct MotorAnglesDeg {
  double m1_deg = 0.0;
  double m2_deg = 0.0;
};

struct JointAnglesDeg {
  double pitch_deg = 0.0; // alpha
  double roll_deg  = 0.0; // beta
};

struct FKReport {
  bool ok = false;
  int iters = 0;
  double final_residual = 0.0;
  JointAnglesDeg joint_deg;
};

struct IKReport {
  bool ok = false;
  // 两支解：每个电机都对应 (phi - acos(rho)) 与 (phi + acos(rho))
  // 这里输出两套组合：minus/minus 与 plus/plus（足够用于回代对比）
  MotorAnglesDeg sol_minus;
  MotorAnglesDeg sol_plus;
  double rho1 = 0.0;
  double rho2 = 0.0;
};

struct AnkleParams {
  std::string name;

  Vector3d A1, B1, C1;
  Vector3d A2, B2, C2;
  Vector3d Ow;

  double Len_Long  = 0.0;
  double Len_Short = 0.0;

  // “电机坐标系 -> 连杆几何坐标系”的符号修正
  bool invert_motor1 = false;
  bool invert_motor2 = false;
};

// --- 参数获取 ---
AnkleParams make_params_v2();


// --- 基础旋转与导数 ---
Matrix3d Rx(double b);
Matrix3d Ry(double a);
Matrix3d dRx(double b);
Matrix3d dRy(double a);

// --- 约束方程 f(alpha,beta)=0 与雅可比 df/d(alpha,beta) ---
FwdOut2D computeFwd(const AnkleParams& P, double alpha, double beta,
                    double th1_link, double th2_link);

// --- FK: motor(deg) -> (alpha,beta)(deg) ---
FKReport forwardKinematics(const AnkleParams& P, double motor1_deg, double motor2_deg,
                           double alpha0_rad = 0.0, double beta0_rad = 0.0,
                           int max_iter = 50, double tol = 1e-12);

// --- IK: (alpha,beta)(deg) -> motor(deg) ---
IKReport inverseKinematics(const AnkleParams& P, double alpha_deg, double beta_deg);
// ================= Jacobian (velocity/torque mapping) =================
// J_link  : d(theta_link)/d(q)   where q=[alpha,beta], theta_link=[th1,th2] in link-geometry coordinates
// J_motor : d(motor)/d(q)        where motor angles are in motor coordinates (after applying invert flags)

Eigen::Matrix2d buildJacIK_link(const AnkleParams& P,
                                double alpha_rad, double beta_rad,
                                double th1_link_rad, double th2_link_rad);

Eigen::Matrix2d buildJacIK_motor(const AnkleParams& P,
                                 double alpha_rad, double beta_rad,
                                 double m1_motor_rad, double m2_motor_rad);

// -------- Velocity layer --------
// IK vel:  (alpha_dot,beta_dot) -> (motor1_dot,motor2_dot)
Eigen::Vector2d ikVelocity_motor(const AnkleParams& P,
                                 double alpha_rad, double beta_rad,
                                 double m1_motor_rad, double m2_motor_rad,
                                 double alpha_dot_rad, double beta_dot_rad);

// FK vel:  (motor1_dot,motor2_dot) -> (alpha_dot,beta_dot)
Eigen::Vector2d fkVelocity_joint(const AnkleParams& P,
                                 double alpha_rad, double beta_rad,
                                 double m1_motor_rad, double m2_motor_rad,
                                 double m1_dot_rad, double m2_dot_rad);

// -------- Torque layer --------
// IK torque: (tau_alpha,tau_beta) -> (tau_m1,tau_m2)
// 说明：这里默认使用 tau_motor = J_motor^{-T} * tau_joint（与 calLinksTrans 的“正号版本”一致）
// 若你要复现 FwdLinks 里那种带负号的约定，把 torque_sign 传 -1 即可。
Eigen::Vector2d ikTorque_motor(const AnkleParams& P,
                               double alpha_rad, double beta_rad,
                               double m1_motor_rad, double m2_motor_rad,
                               double tau_alpha, double tau_beta,
                               double torque_sign = +1.0);

// FK torque: (tau_m1,tau_m2) -> (tau_alpha,tau_beta)
Eigen::Vector2d fkTorque_joint(const AnkleParams& P,
                               double alpha_rad, double beta_rad,
                               double m1_motor_rad, double m2_motor_rad,
                               double tau_m1, double tau_m2,
                               double torque_sign = +1.0);

// --- 工具函数 ---
double deg2rad(double d);
double rad2deg(double r);
double wrapToPi(double x);

} // namespace ankle
