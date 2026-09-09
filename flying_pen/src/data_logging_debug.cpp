// data_logging_debug.cpp
// 목적: suWrenchObs SI 디버그 로그를 별도 CSV + Float64MultiArray로 저장한다.

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <crazyflie_interfaces/msg/log_data_generic.hpp>
#include <crazyflie_interfaces/msg/position.hpp>
#include <crazyflie_interfaces/msg/position_control.hpp>
#include <crazyflie_interfaces/msg/status.hpp>
#include <motion_capture_tracking_interfaces/msg/named_pose_array.hpp>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>

using std::placeholders::_1;

static std::string expand_user_debug(const std::string & path)
{
  if (!path.empty() && path[0] == '~') {
    const char * home = std::getenv("HOME");
    if (home) {
      return std::string(home) + path.substr(1);
    }
  }
  return path;
}

static std::string now_mmddhhmm_debug()
{
  std::time_t t = std::time(nullptr);
  std::tm tm{};
#if defined(_WIN32)
  localtime_s(&tm, &t);
#else
  localtime_r(&t, &tm);
#endif
  char buf[64];
  std::strftime(buf, sizeof(buf), "%m%d%H%M", &tm);
  return std::string(buf);
}

static inline double qnan_debug()
{
  return std::numeric_limits<double>::quiet_NaN();
}

static std::string sanitize_filename_component_debug(const std::string & text)
{
  std::string out;
  out.reserve(text.size());
  for (const char c : text) {
    const bool ok =
      (c >= '0' && c <= '9') ||
      (c >= 'a' && c <= 'z') ||
      (c >= 'A' && c <= 'Z') ||
      c == '_' || c == '-' || c == '.';
    out.push_back(ok ? c : '_');
  }
  return out;
}

class DataLoggingDebugNode : public rclcpp::Node
{
public:
  //  0.. 5 : pose_x y z roll pitch yaw
  //  6.. 9 : cmd_x y z yaw [m, rad]
  // 10..12 : fw_cmd_x y z [m]
  // 13..16 : thrust_f1 f2 f3 f4 [N]
  // 17..20 : pwm_1 2 3 4 [ratio]
  // 21..23 : body_force xyz [N]
  // 24..26 : world_force xyz [N]
  // 27..29 : body_torque xyz [N*m]
  // 30..32 : state_vel xyz [m/s]
  // 33..35 : pos_vel xyz [m/s]
  // 36..38 : acc xyz [m/s^2]
  // 39..41 : vel_des xyz [m/s]
  // 42..44 : att_des rpy [deg]
  // 45     : status_battery_voltage [V]
  // 46     : pm_vbat [V]
  // 47     : zero_bias_count
  // 48..50 : mob_force_none xyz [N]
  // 51..53 : mob_force_residual xyz [N]
  // 54..56 : mob_force_final xyz [N]
  // 57..59 : mob_torque xyz [N*m]
  // 60..62 : mob_residual xyz [N*m]
  // 63..65 : body-frame accel xyz [G], after manual bias correction and before gravity-trim/LPF
  // 66..68 : body-frame gyro xyz [deg/s], Mahony/complementary gyro input
  // 69     : force_desired [N], scalar preload force command from PositionControl
  // 70..72 : normal_preproj xyz [-], normalized force-direction evidence
  // 73..75 : normal_postproj xyz [-], velocity-projected normal candidate
  // 76..78 : normal_estimation xyz [-], estimated world normal vector
  // 79..81 : ee_vel_used xyz [m/s], 1 Hz LPF contact/end-effector velocity used in normal estimation
  // 82     : omega_n [1/s], 1 Hz LPF norm of d/dt(normal_est)
  // 83     : normal_velocity_leakage [m/s], 1 Hz LPF |n_hat^T v_EE|
  // 84     : stabilizer loop elapsed time [us]
  // 85     : stabilizer loop elapsed time max since boot [us]
  // 86     : alpha_frame [-], tangential command gating factor
  // 87     : t1_cmd_des [m/s], gated desired tangential command in t1
  // 88     : t2_cmd_des [m/s], gated desired tangential command in t2
  // 89..91 : tilted_wall position xyz [m], world frame
  // 92..95 : tilted_wall orientation xyzw [-], world frame
  // 96     : firmware thrust effectiveness eta_hat [-]
  // 97..99 : firmware matched force xyz [N], world frame
  // 100..102 : firmware point-contact torque residual xyz [N*m], world frame
  // 103..105 : firmware eta-corrected force xyz [N], world frame (legacy alias)
  // New pipeline fields are append-only; indices 0..105 remain byte-for-byte compatible.
  // 106..108 : rawMobF xyz [N], world frame
  // 109..111 : rawMobT xyz [N*m], world frame
  // 112..114 : contactF xyz [N], world frame
  // 115      : etaHat [-]
  enum DebugIndex : std::size_t {
    IDX_RAW_MOB_FX = 106,
    IDX_RAW_MOB_FY,
    IDX_RAW_MOB_FZ,
    IDX_RAW_MOB_TX,
    IDX_RAW_MOB_TY,
    IDX_RAW_MOB_TZ,
    IDX_CONTACT_FX,
    IDX_CONTACT_FY,
    IDX_CONTACT_FZ,
    IDX_ETA_HAT,
    DEBUG_DATA_SIZE
  };
  static constexpr std::size_t kDataLen = DEBUG_DATA_SIZE;
  static_assert(IDX_RAW_MOB_FX == 106, "new fields must remain append-only");

  DataLoggingDebugNode()
  : Node("data_logging_debug")
  {
    csv_dir_ = expand_user_debug(this->declare_parameter<std::string>(
      "csv_dir", "~/hitl_ws/src/flying_pen/bag/logging"));
    publish_topic_ = this->declare_parameter<std::string>("publish_topic", "/data_logging_msg_debug");
    cf_ns_ = this->declare_parameter<std::string>("cf_ns", "/cf2");
    loop_hz_ = this->declare_parameter<double>("loop_hz", 50.0);

    std::filesystem::create_directories(csv_dir_);
    csv_path_ = (std::filesystem::path(csv_dir_) / (now_mmddhhmm_debug() + "_debug.csv")).string();
    csv_.open(csv_path_, std::ios::out | std::ios::trunc);
    if (!csv_.is_open()) {
      RCLCPP_ERROR(get_logger(), "Failed to open CSV file: %s", csv_path_.c_str());
    } else {
      write_csv_header();
      RCLCPP_INFO(get_logger(), "CSV logging enabled: %s", csv_path_.c_str());
    }

    data_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(publish_topic_, 10);

    auto sensor_qos = rclcpp::QoS(
      rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 10),
      rmw_qos_profile_sensor_data);

    sub_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      cf_ns_ + "/pose", 10, std::bind(&DataLoggingDebugNode::poseCallback, this, _1));
    sub_named_poses_ = this->create_subscription<motion_capture_tracking_interfaces::msg::NamedPoseArray>(
      "/poses", sensor_qos, std::bind(&DataLoggingDebugNode::namedPosesCallback, this, _1));
    sub_cmd_position_ = this->create_subscription<crazyflie_interfaces::msg::Position>(
      cf_ns_ + "/cmd_position", 10, std::bind(&DataLoggingDebugNode::cmdPositionCallback, this, _1));
    sub_cmd_position_control_ = this->create_subscription<crazyflie_interfaces::msg::PositionControl>(
      cf_ns_ + "/cmd_position_control", 10, std::bind(&DataLoggingDebugNode::cmdPositionControlCallback, this, _1));
    sub_ctrl_misc_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      cf_ns_ + "/cf_ctrl_misc", 10, std::bind(&DataLoggingDebugNode::ctrlMiscCallback, this, _1));
    sub_status_ = this->create_subscription<crazyflie_interfaces::msg::Status>(
      cf_ns_ + "/status", 10, std::bind(&DataLoggingDebugNode::statusCallback, this, _1));
    sub_motor_thrust_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      cf_ns_ + "/cf_su_motor_thrust", 10, std::bind(&DataLoggingDebugNode::motorThrustCallback, this, _1));
    sub_motor_pwm_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      cf_ns_ + "/cf_su_motor_pwm", 10, std::bind(&DataLoggingDebugNode::motorPwmCallback, this, _1));
    sub_body_force_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      cf_ns_ + "/cf_su_body_force", 10, std::bind(&DataLoggingDebugNode::bodyForceCallback, this, _1));
    sub_world_force_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      cf_ns_ + "/cf_su_world_force", 10, std::bind(&DataLoggingDebugNode::worldForceCallback, this, _1));
    sub_body_torque_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      cf_ns_ + "/cf_su_body_torque", 10, std::bind(&DataLoggingDebugNode::bodyTorqueCallback, this, _1));
    sub_vel_pair_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      cf_ns_ + "/cf_su_vel_pair", 10, std::bind(&DataLoggingDebugNode::velPairCallback, this, _1));
    sub_acc_normal_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      cf_ns_ + "/cf_su_acc_normal", 10, std::bind(&DataLoggingDebugNode::accNormalCallback, this, _1));
    sub_normal_debug_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      cf_ns_ + "/cf_su_normal_debug", 10, std::bind(&DataLoggingDebugNode::normalDebugCallback, this, _1));
    sub_mob_raw_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      cf_ns_ + "/cf_mob_raw", 10, std::bind(&DataLoggingDebugNode::mobRawCallback, this, _1));
    sub_contact_force_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      cf_ns_ + "/cf_contact_force", 10, std::bind(&DataLoggingDebugNode::contactForceCallback, this, _1));
    sub_normal_metrics_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      cf_ns_ + "/cf_su_normal_metrics", 10, std::bind(&DataLoggingDebugNode::normalMetricsCallback, this, _1));
    sub_stabilizer_timing_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      cf_ns_ + "/cf_stabilizer_timing", 10, std::bind(&DataLoggingDebugNode::stabilizerTimingCallback, this, _1));
    sub_alpha_frame_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      cf_ns_ + "/cf_su_alpha_frame", 10, std::bind(&DataLoggingDebugNode::alphaFrameCallback, this, _1));
    sub_imu_raw_pair_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      cf_ns_ + "/cf_imu_raw_pair", 10, std::bind(&DataLoggingDebugNode::imuRawPairCallback, this, _1));
    sub_vel_att_des_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      cf_ns_ + "/vel_att_des", 10, std::bind(&DataLoggingDebugNode::velAttDesCallback, this, _1));
    sub_filename_tag_ = this->create_subscription<std_msgs::msg::String>(
      "/flying_pen/debug_log_filename_tag", 10,
      std::bind(&DataLoggingDebugNode::filenameTagCallback, this, _1));

    RCLCPP_INFO(get_logger(), "data_logging_debug node started");
  }

  ~DataLoggingDebugNode() override
  {
    if (csv_.is_open()) {
      csv_.flush();
      csv_.close();
    }
  }

  double loop_hz() const { return loop_hz_; }

  void loopOnce()
  {
    std_msgs::msg::Float64MultiArray out;
    out.data.reserve(kDataLen);

    push3(out, pose_xyz_);
    push3(out, pose_rpy_);
    push4(out, cmd_xyzyaw_);
    push3(out, fw_cmd_xyz_);
    push4(out, motor_thrust_);
    push4(out, motor_pwm_);
    push3(out, body_force_);
    push3(out, world_force_);
    push3(out, body_torque_);
    push3(out, state_vel_);
    push3(out, pos_vel_);
    push3(out, acc_);
    push3(out, vel_des_);
    push3(out, att_des_);
    out.data.push_back(status_batt_v_);
    out.data.push_back(pm_vbat_);
    out.data.push_back(zero_bias_count_);
    push3(out, mob_force_none_);
    push3(out, mob_force_residual_);
    push3(out, mob_force_final_);
    push3(out, mob_torque_);
    push3(out, mob_residual_);
    push3(out, acc_raw_body_);
    push3(out, gyro_raw_body_);
    out.data.push_back(force_desired_);
    push3(out, normal_preproj_);
    push3(out, normal_postproj_);
    push3(out, normal_est_);
    push3(out, ee_vel_used_);
    out.data.push_back(omega_n_);
    out.data.push_back(normal_velocity_leakage_);
    out.data.push_back(stabilizer_loop_dt_us_);
    out.data.push_back(stabilizer_loop_dt_us_max_);
    out.data.push_back(alpha_frame_);
    out.data.push_back(t1_cmd_des_);
    out.data.push_back(t2_cmd_des_);
    push3(out, wall_xyz_);
    push4(out, wall_quat_xyzw_);
    out.data.push_back(thrust_eff_eta_hat_);
    push3(out, thrust_eff_match_force_);
    push3(out, thrust_eff_residual_);
    push3(out, thrust_eff_corrected_force_);
    push3(out, raw_mob_force_);
    push3(out, raw_mob_torque_);
    push3(out, contact_force_);
    out.data.push_back(eta_hat_);

    if (out.data.size() != static_cast<size_t>(kDataLen)) {
      out.data.resize(kDataLen, qnan_debug());
    }

    data_pub_->publish(out);
    log_csv_row(out);
  }

private:
  static void push3(std_msgs::msg::Float64MultiArray & m, const std::array<double, 3> & a)
  {
    m.data.push_back(a[0]);
    m.data.push_back(a[1]);
    m.data.push_back(a[2]);
  }

  static void push4(std_msgs::msg::Float64MultiArray & m, const std::array<double, 4> & a)
  {
    m.data.push_back(a[0]);
    m.data.push_back(a[1]);
    m.data.push_back(a[2]);
    m.data.push_back(a[3]);
  }

  void copy3(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg, std::array<double, 3> & dst)
  {
    if (msg->values.size() >= 3) {
      dst[0] = msg->values[0];
      dst[1] = msg->values[1];
      dst[2] = msg->values[2];
    }
  }

  void copy4(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg, std::array<double, 4> & dst)
  {
    if (msg->values.size() >= 4) {
      dst[0] = msg->values[0];
      dst[1] = msg->values[1];
      dst[2] = msg->values[2];
      dst[3] = msg->values[3];
    }
  }

  void write_csv_header()
  {
    csv_
      << "pose_x,pose_y,pose_z,pose_roll,pose_pitch,pose_yaw,"
      << "cmd_x,cmd_y,cmd_z,cmd_yaw,"
      << "fwCmd_x,fwCmd_y,fwCmd_z,"
      << "f1,f2,f3,f4,"
      << "pwm1,pwm2,pwm3,pwm4,"
      << "bodyFx,bodyFy,bodyFz,"
      << "worldFx,worldFy,worldFz,"
      << "bodyTx,bodyTy,bodyTz,"
      << "stateVx,stateVy,stateVz,"
      << "posVx,posVy,posVz,"
      << "accWx,accWy,accWz,"
      << "velDes_vx,velDes_vy,velDes_vz,"
      << "attDes_roll,attDes_pitch,attDes_yaw,"
      << "status_battery_voltage,pm_vbat,"
      << "zero_bias_count,"
      << "mobForceNone_x,mobForceNone_y,mobForceNone_z,"
      << "mobForceResidual_x,mobForceResidual_y,mobForceResidual_z,"
      << "mobForceFinal_x,mobForceFinal_y,mobForceFinal_z,"
      << "mobTorque_x,mobTorque_y,mobTorque_z,"
      << "mobResidual_x,mobResidual_y,mobResidual_z,"
      << "accRawBody_x,accRawBody_y,accRawBody_z,"
      << "gyroBody_x,gyroBody_y,gyroBody_z,"
      << "forceDesired,"
      << "normalPre_x,normalPre_y,normalPre_z,"
      << "normalPost_x,normalPost_y,normalPost_z,"
      << "normalEst_x,normalEst_y,normalEst_z,"
      << "eeVelUsed_x,eeVelUsed_y,eeVelUsed_z,"
      << "omega_n,normalVelocityLeakage,"
      << "loopDtUs,loopDtUsMax,"
      << "alphaFrame,t1CmdDes,t2CmdDes,"
      << "wall_x,wall_y,wall_z,"
      << "wall_qx,wall_qy,wall_qz,wall_qw,"
      << "thrustEffEtaHat,"
      << "thrustEffMatchFx,thrustEffMatchFy,thrustEffMatchFz,"
      << "thrustEffEpsTx,thrustEffEpsTy,thrustEffEpsTz,"
      << "thrustEffCorrFx,thrustEffCorrFy,thrustEffCorrFz,"
      << "rawMobFx,rawMobFy,rawMobFz,"
      << "rawMobTx,rawMobTy,rawMobTz,"
      << "contactFx,contactFy,contactFz,etaHat\n";
    csv_.flush();
  }

  void log_csv_row(const std_msgs::msg::Float64MultiArray & msg)
  {
    if (!csv_.is_open()) {
      return;
    }

    csv_ << std::setprecision(10) << std::fixed;
    for (size_t i = 0; i < msg.data.size(); ++i) {
      if (i > 0) {
        csv_ << ",";
      }
      csv_ << msg.data[i];
    }
    csv_ << "\n";
    if (++csv_line_count_ % 100 == 0) {
      csv_.flush();
    }
  }

  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    pose_xyz_[0] = msg->pose.position.x;
    pose_xyz_[1] = msg->pose.position.y;
    pose_xyz_[2] = msg->pose.position.z;

    tf2::Quaternion q(
      msg->pose.orientation.x,
      msg->pose.orientation.y,
      msg->pose.orientation.z,
      msg->pose.orientation.w);
    q.normalize();
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    pose_rpy_[0] = roll;
    pose_rpy_[1] = pitch;
    pose_rpy_[2] = yaw;
  }

  void namedPosesCallback(
    const motion_capture_tracking_interfaces::msg::NamedPoseArray::SharedPtr msg)
  {
    for (const auto & named_pose : msg->poses) {
      if (named_pose.name != "tilted_wall") {
        continue;
      }

      const auto & p = named_pose.pose.position;
      const auto & q = named_pose.pose.orientation;
      if (
        !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
        !std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w))
      {
        return;
      }

      wall_xyz_[0] = named_pose.pose.position.x;
      wall_xyz_[1] = named_pose.pose.position.y;
      wall_xyz_[2] = named_pose.pose.position.z;
      wall_quat_xyzw_[0] = named_pose.pose.orientation.x;
      wall_quat_xyzw_[1] = named_pose.pose.orientation.y;
      wall_quat_xyzw_[2] = named_pose.pose.orientation.z;
      wall_quat_xyzw_[3] = named_pose.pose.orientation.w;
      return;
    }
  }

  void motorThrustCallback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg) { copy4(msg, motor_thrust_); }
  void motorPwmCallback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg) { copy4(msg, motor_pwm_); }
  void bodyForceCallback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg) { copy3(msg, body_force_); }
  void worldForceCallback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg) { copy3(msg, world_force_); }
  void bodyTorqueCallback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg) { copy3(msg, body_torque_); }
  void velPairCallback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg)
  {
    if (msg->values.size() >= 6) {
      state_vel_[0] = msg->values[0];
      state_vel_[1] = msg->values[1];
      state_vel_[2] = msg->values[2];
      pos_vel_[0] = msg->values[3];
      pos_vel_[1] = msg->values[4];
      pos_vel_[2] = msg->values[5];
    }
  }
  void accNormalCallback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg)
  {
    if (msg->values.size() >= 6) {
      acc_[0] = msg->values[0];
      acc_[1] = msg->values[1];
      acc_[2] = msg->values[2];
      normal_est_[0] = msg->values[3];
      normal_est_[1] = msg->values[4];
      normal_est_[2] = msg->values[5];
    }
  }
  void normalDebugCallback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg)
  {
    if (msg->values.size() >= 6) {
      normal_preproj_[0] = msg->values[0];
      normal_preproj_[1] = msg->values[1];
      normal_preproj_[2] = msg->values[2];
      normal_postproj_[0] = msg->values[3];
      normal_postproj_[1] = msg->values[4];
      normal_postproj_[2] = msg->values[5];
    }
  }
  void mobRawCallback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg)
  {
    if (msg->values.size() >= 6) {
      for (std::size_t i = 0; i < 3; ++i) {
        raw_mob_force_[i] = msg->values[i];
        raw_mob_torque_[i] = msg->values[i + 3];
      }
      // Preserve useful legacy aliases without changing their indices.
      mob_force_final_ = raw_mob_force_;
      mob_torque_ = raw_mob_torque_;
    }
  }
  void contactForceCallback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg)
  {
    if (msg->values.size() >= 4) {
      for (std::size_t i = 0; i < 3; ++i) {
        contact_force_[i] = msg->values[i];
      }
      eta_hat_ = msg->values[3];
      // Preserve the pre-existing eta/corrected-force columns as aliases.
      thrust_eff_corrected_force_ = contact_force_;
      thrust_eff_eta_hat_ = eta_hat_;
    }
  }
  void normalMetricsCallback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg)
  {
    if (msg->values.size() >= 5) {
      ee_vel_used_[0] = msg->values[0];
      ee_vel_used_[1] = msg->values[1];
      ee_vel_used_[2] = msg->values[2];
      omega_n_ = msg->values[3];
      normal_velocity_leakage_ = msg->values[4];
    }
  }
  void stabilizerTimingCallback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg)
  {
    if (msg->values.size() >= 2) {
      stabilizer_loop_dt_us_ = msg->values[0];
      stabilizer_loop_dt_us_max_ = msg->values[1];
    }
  }
  void alphaFrameCallback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg)
  {
    if (msg->values.size() >= 3) {
      alpha_frame_ = msg->values[0];
      t1_cmd_des_ = msg->values[1];
      t2_cmd_des_ = msg->values[2];
    }
  }
  void imuRawPairCallback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg)
  {
    if (msg->values.size() >= 6) {
      acc_raw_body_[0] = msg->values[0];
      acc_raw_body_[1] = msg->values[1];
      acc_raw_body_[2] = msg->values[2];
      gyro_raw_body_[0] = msg->values[3];
      gyro_raw_body_[1] = msg->values[4];
      gyro_raw_body_[2] = msg->values[5];
    }
  }
  void velAttDesCallback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg)
  {
    if (msg->values.size() >= 6) {
      vel_des_[0] = msg->values[0];
      vel_des_[1] = msg->values[1];
      vel_des_[2] = msg->values[2];
      att_des_[0] = msg->values[3];
      att_des_[1] = msg->values[4];
      att_des_[2] = msg->values[5];
    }
  }
  void cmdPositionCallback(const crazyflie_interfaces::msg::Position::SharedPtr msg)
  {
    cmd_xyzyaw_[0] = msg->x;
    cmd_xyzyaw_[1] = msg->y;
    cmd_xyzyaw_[2] = msg->z;
    cmd_xyzyaw_[3] = msg->yaw;
  }
  void cmdPositionControlCallback(const crazyflie_interfaces::msg::PositionControl::SharedPtr msg)
  {
    force_desired_ = msg->force_desired;
  }
  void statusCallback(const crazyflie_interfaces::msg::Status::SharedPtr msg)
  {
    status_batt_v_ = msg->battery_voltage;
  }
  void ctrlMiscCallback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg)
  {
    if (msg->values.size() >= 4) {
      pm_vbat_ = msg->values[0];
      fw_cmd_xyz_[0] = msg->values[1];
      fw_cmd_xyz_[1] = msg->values[2];
      fw_cmd_xyz_[2] = msg->values[3];
    }
  }

  void filenameTagCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    if (!msg || msg->data.empty() || csv_path_.empty()) {
      return;
    }

    const std::filesystem::path current_path(csv_path_);
    const std::filesystem::path parent_dir = current_path.parent_path();
    const std::string sanitized_tag = sanitize_filename_component_debug(msg->data);
    if (sanitized_tag.empty()) {
      RCLCPP_WARN(get_logger(), "Ignoring empty debug log filename tag");
      return;
    }

    const std::string base_name = current_path.filename().string();
    const std::string suffix = "_debug";
    const std::size_t suffix_pos = base_name.find(suffix);
    const std::string prefix =
      (suffix_pos == std::string::npos) ? now_mmddhhmm_debug() : base_name.substr(0, suffix_pos);
    const std::filesystem::path new_path =
      parent_dir / (prefix + "_debug_" + sanitized_tag + ".csv");

    if (new_path == current_path) {
      return;
    }

    if (csv_.is_open()) {
      csv_.flush();
      csv_.close();
    }

    std::error_code ec;
    std::filesystem::rename(current_path, new_path, ec);
    if (ec) {
      RCLCPP_WARN(
        get_logger(),
        "Failed to rename debug CSV from %s to %s: %s",
        current_path.string().c_str(),
        new_path.string().c_str(),
        ec.message().c_str());
      csv_.open(csv_path_, std::ios::out | std::ios::app);
      return;
    }

    csv_path_ = new_path.string();
    csv_.open(csv_path_, std::ios::out | std::ios::app);
    if (!csv_.is_open()) {
      RCLCPP_ERROR(get_logger(), "Failed to reopen renamed CSV file: %s", csv_path_.c_str());
      return;
    }

    RCLCPP_INFO(get_logger(), "Debug CSV renamed to: %s", csv_path_.c_str());
  }

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr data_pub_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_;
  rclcpp::Subscription<motion_capture_tracking_interfaces::msg::NamedPoseArray>::SharedPtr sub_named_poses_;
  rclcpp::Subscription<crazyflie_interfaces::msg::Position>::SharedPtr sub_cmd_position_;
  rclcpp::Subscription<crazyflie_interfaces::msg::PositionControl>::SharedPtr sub_cmd_position_control_;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr sub_ctrl_misc_;
  rclcpp::Subscription<crazyflie_interfaces::msg::Status>::SharedPtr sub_status_;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr sub_motor_thrust_;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr sub_motor_pwm_;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr sub_body_force_;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr sub_world_force_;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr sub_body_torque_;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr sub_vel_pair_;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr sub_acc_normal_;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr sub_normal_debug_;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr sub_mob_raw_;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr sub_contact_force_;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr sub_normal_metrics_;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr sub_stabilizer_timing_;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr sub_alpha_frame_;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr sub_imu_raw_pair_;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr sub_vel_att_des_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_filename_tag_;

  std::string csv_dir_;
  std::string csv_path_;
  std::ofstream csv_;
  uint64_t csv_line_count_{0};

  std::string publish_topic_;
  std::string cf_ns_;
  double loop_hz_{50.0};

  std::array<double, 3> pose_xyz_ = {qnan_debug(), qnan_debug(), qnan_debug()};
  std::array<double, 3> pose_rpy_ = {qnan_debug(), qnan_debug(), qnan_debug()};
  std::array<double, 4> cmd_xyzyaw_ = {qnan_debug(), qnan_debug(), qnan_debug(), qnan_debug()};
  std::array<double, 3> fw_cmd_xyz_ = {qnan_debug(), qnan_debug(), qnan_debug()};
  std::array<double, 4> motor_thrust_ = {qnan_debug(), qnan_debug(), qnan_debug(), qnan_debug()};
  std::array<double, 4> motor_pwm_ = {qnan_debug(), qnan_debug(), qnan_debug(), qnan_debug()};
  std::array<double, 3> body_force_ = {qnan_debug(), qnan_debug(), qnan_debug()};
  std::array<double, 3> world_force_ = {qnan_debug(), qnan_debug(), qnan_debug()};
  std::array<double, 3> body_torque_ = {qnan_debug(), qnan_debug(), qnan_debug()};
  std::array<double, 3> state_vel_ = {qnan_debug(), qnan_debug(), qnan_debug()};
  std::array<double, 3> pos_vel_ = {qnan_debug(), qnan_debug(), qnan_debug()};
  std::array<double, 3> acc_ = {qnan_debug(), qnan_debug(), qnan_debug()};
  std::array<double, 3> acc_raw_body_ = {qnan_debug(), qnan_debug(), qnan_debug()};
  std::array<double, 3> gyro_raw_body_ = {qnan_debug(), qnan_debug(), qnan_debug()};
  std::array<double, 3> vel_des_ = {qnan_debug(), qnan_debug(), qnan_debug()};
  std::array<double, 3> att_des_ = {qnan_debug(), qnan_debug(), qnan_debug()};
  double status_batt_v_ = qnan_debug();
  double pm_vbat_ = qnan_debug();
  double zero_bias_count_ = qnan_debug();
  std::array<double, 3> mob_force_none_ = {qnan_debug(), qnan_debug(), qnan_debug()};
  std::array<double, 3> mob_force_residual_ = {qnan_debug(), qnan_debug(), qnan_debug()};
  std::array<double, 3> mob_force_final_ = {qnan_debug(), qnan_debug(), qnan_debug()};
  std::array<double, 3> mob_torque_ = {qnan_debug(), qnan_debug(), qnan_debug()};
  std::array<double, 3> mob_residual_ = {qnan_debug(), qnan_debug(), qnan_debug()};
  std::array<double, 3> normal_preproj_ = {qnan_debug(), qnan_debug(), qnan_debug()};
  std::array<double, 3> normal_postproj_ = {qnan_debug(), qnan_debug(), qnan_debug()};
  std::array<double, 3> normal_est_ = {qnan_debug(), qnan_debug(), qnan_debug()};
  double thrust_eff_eta_hat_ = qnan_debug();
  std::array<double, 3> thrust_eff_match_force_ = {qnan_debug(), qnan_debug(), qnan_debug()};
  std::array<double, 3> thrust_eff_residual_ = {qnan_debug(), qnan_debug(), qnan_debug()};
  std::array<double, 3> thrust_eff_corrected_force_ = {qnan_debug(), qnan_debug(), qnan_debug()};
  std::array<double, 3> raw_mob_force_ = {qnan_debug(), qnan_debug(), qnan_debug()};
  std::array<double, 3> raw_mob_torque_ = {qnan_debug(), qnan_debug(), qnan_debug()};
  std::array<double, 3> contact_force_ = {qnan_debug(), qnan_debug(), qnan_debug()};
  double eta_hat_ = qnan_debug();
  std::array<double, 3> ee_vel_used_ = {qnan_debug(), qnan_debug(), qnan_debug()};
  double omega_n_ = qnan_debug();
  double normal_velocity_leakage_ = qnan_debug();
  double stabilizer_loop_dt_us_ = qnan_debug();
  double stabilizer_loop_dt_us_max_ = qnan_debug();
  double alpha_frame_ = qnan_debug();
  double t1_cmd_des_ = qnan_debug();
  double t2_cmd_des_ = qnan_debug();
  double force_desired_ = qnan_debug();
  std::array<double, 3> wall_xyz_ = {qnan_debug(), qnan_debug(), qnan_debug()};
  std::array<double, 4> wall_quat_xyzw_ = {
    qnan_debug(), qnan_debug(), qnan_debug(), qnan_debug()};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DataLoggingDebugNode>();
  rclcpp::Rate rate(node->loop_hz());

  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    node->loopOnce();
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
