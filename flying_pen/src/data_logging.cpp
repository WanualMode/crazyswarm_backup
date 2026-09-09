// data_logging.cpp
// 목적: crazyflies.yaml의 현재 firmware_logging 설정에 맞춰
//       실제 비행 로그를 CSV + Float64MultiArray로 저장한다.

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <string>
#include <array>

#include <crazyflie_interfaces/msg/log_data_generic.hpp>
#include <crazyflie_interfaces/msg/position.hpp>
#include <crazyflie_interfaces/msg/status.hpp>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

using std::placeholders::_1;

static std::string expand_user(const std::string& path)
{
  if (!path.empty() && path[0] == '~') {
    const char* home = std::getenv("HOME");
    if (home) {
      return std::string(home) + path.substr(1);
    }
  }
  return path;
}

static std::string now_mmddhhmm()
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

static inline double qnan() { return std::numeric_limits<double>::quiet_NaN(); }

class DataLoggingNode : public rclcpp::Node
{
public:
  // Packed output layout
  //  0.. 5 : pose_x y z roll pitch yaw
  //  6     : status_battery_voltage
  //  7.. 8 : raw_battery_voltage filt_battery_voltage
  //  9..12 : cmd_x cmd_y cmd_z cmd_yaw
  // 13..15 : fwCmd_x fwCmd_y fwCmd_z
  // 16..18 : est_vx est_vy est_vz
  // 19..21 : est_ax est_ay est_az
  // 22..24 : gyro_x gyro_y gyro_z
  // 25..27 : angAcc_x angAcc_y angAcc_z
  // 28..30 : velDes_vx velDes_vy velDes_vz
  // 31..33 : attDes_roll attDes_pitch attDes_yaw
  // 34..37 : motor_f1 motor_f2 motor_f3 motor_f4
  // 38..41 : motor_f1_scaled motor_f2_scaled motor_f3_scaled motor_f4_scaled
  // 42..44 : bodyInFx bodyInFy bodyInFz
  // 45..47 : droneWorldFx droneWorldFy droneWorldFz
  // 48..50 : droneWorldFx_scaled droneWorldFy_scaled droneWorldFz_scaled
  // 51     : zero_bias_count
  // 52..54 : rateDes_roll rateDes_pitch rateDes_yaw
  static constexpr int kDataLen = 55;

  DataLoggingNode()
  : Node("data_logging")
  {
    RCLCPP_INFO(get_logger(), "data_logging node started");

    csv_dir_ = expand_user(this->declare_parameter<std::string>(
      "csv_dir", "~/hitl_ws/src/flying_pen/bag/logging"));
    const int flush_every_n_param = static_cast<int>(
      this->declare_parameter<int64_t>("flush_every_n", 200));
    flush_every_n_ = std::max(1, flush_every_n_param);
    publish_topic_ = this->declare_parameter<std::string>("publish_topic", "/data_logging_msg");
    cf_ns_ = this->declare_parameter<std::string>("cf_ns", "/cf2");
    loop_hz_ = this->declare_parameter<double>("loop_hz", 100.0);
    stale_warn_sec_ = this->declare_parameter<double>("stale_warn_sec", 0.5);
    stale_fail_sec_ = this->declare_parameter<double>("stale_fail_sec", 2.0);

    try {
      std::filesystem::create_directories(csv_dir_);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Failed to create csv_dir '%s': %s", csv_dir_.c_str(), e.what());
    }

    csv_path_ = (std::filesystem::path(csv_dir_) / (now_mmddhhmm() + ".csv")).string();
    csv_.open(csv_path_, std::ios::out | std::ios::trunc);
    if (!csv_.is_open()) {
      RCLCPP_ERROR(get_logger(), "Failed to open CSV file: %s", csv_path_.c_str());
    } else {
      write_csv_header();
      RCLCPP_INFO(get_logger(), "CSV logging enabled: %s", csv_path_.c_str());
    }

    data_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(publish_topic_, 10);

    sub_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      cf_ns_ + "/pose", 10, std::bind(&DataLoggingNode::poseCallback, this, _1));
    sub_status_ = this->create_subscription<crazyflie_interfaces::msg::Status>(
      cf_ns_ + "/status", 10, std::bind(&DataLoggingNode::statusCallback, this, _1));
    sub_cf_voltage_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      cf_ns_ + "/cf_voltage", 10, std::bind(&DataLoggingNode::cfVoltageCallback, this, _1));
    sub_cmd_position_ = this->create_subscription<crazyflie_interfaces::msg::Position>(
      cf_ns_ + "/cmd_position", 10, std::bind(&DataLoggingNode::cmdPositionCallback, this, _1));
    sub_cf_ctrl_target_pos_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      cf_ns_ + "/cf_ctrl_target_pos", 10, std::bind(&DataLoggingNode::cfCtrlTargetPosCallback, this, _1));
    sub_state_estimate_velocity_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      cf_ns_ + "/stateEstimate_velocity", 10, std::bind(&DataLoggingNode::stateEstimateVelocityCallback, this, _1));
    sub_state_estimate_acc_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      cf_ns_ + "/stateEstimate_acc", 10, std::bind(&DataLoggingNode::stateEstimateAccCallback, this, _1));
    sub_gyro_feedback_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      cf_ns_ + "/gyro_feedback", 10, std::bind(&DataLoggingNode::gyroFeedbackCallback, this, _1));
    sub_vel_des_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      cf_ns_ + "/vel_des", 10, std::bind(&DataLoggingNode::velDesCallback, this, _1));
    sub_att_des_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      cf_ns_ + "/att_des", 10, std::bind(&DataLoggingNode::attDesCallback, this, _1));
    sub_rate_des_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      cf_ns_ + "/rate_des", 10, std::bind(&DataLoggingNode::rateDesCallback, this, _1));
    sub_cf_motor_force_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      cf_ns_ + "/cf_motor_force", 10, std::bind(&DataLoggingNode::cfMotorForceCallback, this, _1));
    sub_cf_motor_force_scaled_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      cf_ns_ + "/cf_motor_force_scaled", 10, std::bind(&DataLoggingNode::cfMotorForceScaledCallback, this, _1));
    sub_cf_body_input_force_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      cf_ns_ + "/cf_body_input_force", 10, std::bind(&DataLoggingNode::cfBodyInputForceCallback, this, _1));
    sub_cf_world_force_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      cf_ns_ + "/cf_world_force", 10, std::bind(&DataLoggingNode::cfWorldForceCallback, this, _1));
    sub_cf_F_input_scaled_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      cf_ns_ + "/cf_F_input_scaled", 10, std::bind(&DataLoggingNode::cfFInputScaledCallback, this, _1));
  }

  ~DataLoggingNode() override
  {
    if (csv_.is_open()) {
      csv_.flush();
      csv_.close();
    }
  }

  double loop_hz() const { return loop_hz_; }

  void loopOnce()
  {
    const double t = now_sec();

    std_msgs::msg::Float64MultiArray out;
    out.data.reserve(kDataLen);

    push3(out, pose_xyz_);
    push3(out, pose_rpy_);
    out.data.push_back(status_batt_v_);
    push2(out, cf_voltage_);
    push4(out, cmd_xyzyaw_);
    push3(out, fw_cmd_xyz_);
    push3(out, state_estimate_vel_);
    push3(out, state_estimate_acc_);
    push3(out, gyro_feedback_);
    push3(out, ang_acc_feedback_);
    push3(out, vel_des_);
    push3(out, att_des_);
    push4(out, motor_force_);
    push4(out, motor_force_scaled_);
    push3(out, cf_body_input_force_);
    push3(out, cf_world_force_);
    push3(out, cf_F_input_scaled_);
    out.data.push_back(zero_bias_count_);
    push3(out, rate_des_);

    if (out.data.size() != static_cast<size_t>(kDataLen)) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Packed length mismatch: got %zu, expected %d", out.data.size(), kDataLen);
      out.data.resize(kDataLen, qnan());
    }

    const uint64_t mask = build_validity_mask(t);
    data_pub_->publish(out);
    log_csv_row(t, out, mask);
    have_published_once_ = true;
    warn_if_stale(t);
  }

private:
  static void push2(std_msgs::msg::Float64MultiArray& m, const std::array<double, 2>& a)
  {
    m.data.push_back(a[0]);
    m.data.push_back(a[1]);
  }

  static void push3(std_msgs::msg::Float64MultiArray& m, const std::array<double, 3>& a)
  {
    m.data.push_back(a[0]);
    m.data.push_back(a[1]);
    m.data.push_back(a[2]);
  }

  static void push4(std_msgs::msg::Float64MultiArray& m, const std::array<double, 4>& a)
  {
    m.data.push_back(a[0]);
    m.data.push_back(a[1]);
    m.data.push_back(a[2]);
    m.data.push_back(a[3]);
  }

  void write_csv_header()
  {
    csv_ << "t_sec";
    for (int i = 0; i < kDataLen; ++i) {
      csv_ << "," << col_name(i);
    }

    csv_ << ",age_pose"
         << ",age_status"
         << ",age_cf_voltage"
         << ",age_cmd_position"
         << ",age_cf_ctrl_target_pos"
         << ",age_stateEstimate_velocity"
         << ",age_stateEstimate_acc"
         << ",age_gyro_feedback"
         << ",age_vel_des"
         << ",age_att_des"
         << ",age_rate_des"
         << ",age_cf_motor_force"
         << ",age_cf_motor_force_scaled"
         << ",age_cf_body_input_force"
         << ",age_cf_world_force"
         << ",age_cf_F_input_scaled"
         << ",age_cf_zero_bias_dbg"
         << ",validity_bitmask\n";
    csv_.flush();
  }

  std::string col_name(int idx) const
  {
    switch (idx) {
      case 0: return "pose_x";
      case 1: return "pose_y";
      case 2: return "pose_z";
      case 3: return "pose_roll";
      case 4: return "pose_pitch";
      case 5: return "pose_yaw";
      case 6: return "status_battery_voltage";
      case 7: return "raw_battery_voltage";
      case 8: return "filt_battery_voltage";
      case 9: return "cmd_x";
      case 10: return "cmd_y";
      case 11: return "cmd_z";
      case 12: return "cmd_yaw";
      case 13: return "fwCmd_x";
      case 14: return "fwCmd_y";
      case 15: return "fwCmd_z";
      case 16: return "est_vx";
      case 17: return "est_vy";
      case 18: return "est_vz";
      case 19: return "est_ax";
      case 20: return "est_ay";
      case 21: return "est_az";
      case 22: return "gyro_x";
      case 23: return "gyro_y";
      case 24: return "gyro_z";
      case 25: return "angAcc_x";
      case 26: return "angAcc_y";
      case 27: return "angAcc_z";
      case 28: return "velDes_vx";
      case 29: return "velDes_vy";
      case 30: return "velDes_vz";
      case 31: return "attDes_roll";
      case 32: return "attDes_pitch";
      case 33: return "attDes_yaw";
      case 34: return "motor_f1";
      case 35: return "motor_f2";
      case 36: return "motor_f3";
      case 37: return "motor_f4";
      case 38: return "motor_f1_scaled";
      case 39: return "motor_f2_scaled";
      case 40: return "motor_f3_scaled";
      case 41: return "motor_f4_scaled";
      case 42: return "bodyInFx";
      case 43: return "bodyInFy";
      case 44: return "bodyInFz";
      case 45: return "droneWorldFx";
      case 46: return "droneWorldFy";
      case 47: return "droneWorldFz";
      case 48: return "droneWorldFx_scaled";
      case 49: return "droneWorldFy_scaled";
      case 50: return "droneWorldFz_scaled";
      case 51: return "zero_bias_count";
      case 52: return "rateDes_roll";
      case 53: return "rateDes_pitch";
      case 54: return "rateDes_yaw";
      default: return "d" + std::to_string(idx);
    }
  }

  void log_csv_row(double t, const std_msgs::msg::Float64MultiArray& msg, uint64_t mask)
  {
    if (!csv_.is_open()) {
      return;
    }

    csv_ << std::setprecision(10) << std::fixed;
    csv_ << t;
    for (int i = 0; i < kDataLen; ++i) {
      const double v = (i < static_cast<int>(msg.data.size())) ? msg.data[i] : qnan();
      csv_ << "," << v;
    }

    csv_ << "," << age_sec(t, t_last_pose_)
         << "," << age_sec(t, t_last_status_)
         << "," << age_sec(t, t_last_cf_voltage_)
         << "," << age_sec(t, t_last_cmd_position_)
         << "," << age_sec(t, t_last_cf_ctrl_target_pos_)
         << "," << age_sec(t, t_last_state_estimate_vel_)
         << "," << age_sec(t, t_last_state_estimate_acc_)
         << "," << age_sec(t, t_last_gyro_feedback_)
         << "," << age_sec(t, t_last_vel_des_)
         << "," << age_sec(t, t_last_att_des_)
         << "," << age_sec(t, t_last_rate_des_)
         << "," << age_sec(t, t_last_cf_motor_force_)
         << "," << age_sec(t, t_last_cf_motor_force_scaled_)
         << "," << age_sec(t, t_last_cf_body_input_force_)
         << "," << age_sec(t, t_last_cf_world_force_)
         << "," << age_sec(t, t_last_cf_F_input_scaled_)
         << "," << age_sec(t, t_last_cf_zero_bias_dbg_)
         << "," << static_cast<unsigned long long>(mask) << "\n";

    ++csv_line_count_;
    if (csv_line_count_ <= 20 || (csv_line_count_ % static_cast<uint64_t>(flush_every_n_) == 0)) {
      csv_.flush();
    }
  }

  static double age_sec(double now, double last)
  {
    if (!std::isfinite(last)) {
      return std::numeric_limits<double>::infinity();
    }
    const double age = now - last;
    return (age < 0.0) ? 0.0 : age;
  }

  uint64_t build_validity_mask(double t)
  {
    // 0: packed size ok
    // 1: published at least once
    // 2..18: topic freshness
    uint64_t mask = 0ull;
    mask |= (1ull << 0);
    if (have_published_once_) {
      mask |= (1ull << 1);
    }
    if (age_sec(t, t_last_pose_) < stale_fail_sec_) mask |= (1ull << 2);
    if (age_sec(t, t_last_status_) < stale_fail_sec_) mask |= (1ull << 3);
    if (age_sec(t, t_last_cf_voltage_) < stale_fail_sec_) mask |= (1ull << 4);
    if (age_sec(t, t_last_cmd_position_) < stale_fail_sec_) mask |= (1ull << 5);
    if (age_sec(t, t_last_cf_ctrl_target_pos_) < stale_fail_sec_) mask |= (1ull << 6);
    if (age_sec(t, t_last_state_estimate_vel_) < stale_fail_sec_) mask |= (1ull << 7);
    if (age_sec(t, t_last_state_estimate_acc_) < stale_fail_sec_) mask |= (1ull << 8);
    if (age_sec(t, t_last_gyro_feedback_) < stale_fail_sec_) mask |= (1ull << 9);
    if (age_sec(t, t_last_vel_des_) < stale_fail_sec_) mask |= (1ull << 10);
    if (age_sec(t, t_last_att_des_) < stale_fail_sec_) mask |= (1ull << 11);
    if (age_sec(t, t_last_rate_des_) < stale_fail_sec_) mask |= (1ull << 12);
    if (age_sec(t, t_last_cf_motor_force_) < stale_fail_sec_) mask |= (1ull << 13);
    if (age_sec(t, t_last_cf_motor_force_scaled_) < stale_fail_sec_) mask |= (1ull << 14);
    if (age_sec(t, t_last_cf_body_input_force_) < stale_fail_sec_) mask |= (1ull << 15);
    if (age_sec(t, t_last_cf_world_force_) < stale_fail_sec_) mask |= (1ull << 16);
    if (age_sec(t, t_last_cf_F_input_scaled_) < stale_fail_sec_) mask |= (1ull << 17);
    // Bit 18 is intentionally left clear: the retired zero-bias topic is no longer subscribed.
    return mask;
  }

  void warn_if_stale(double t)
  {
    auto warn_topic = [&](const char* name, double age) {
      if (age > stale_warn_sec_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Topic stale: %-24s age=%.3fs (warn>%.3fs, fail>%.3fs)",
          name, age, stale_warn_sec_, stale_fail_sec_);
      }
    };

    warn_topic("pose", age_sec(t, t_last_pose_));
    warn_topic("status", age_sec(t, t_last_status_));
    warn_topic("cf_voltage", age_sec(t, t_last_cf_voltage_));
    warn_topic("cmd_position", age_sec(t, t_last_cmd_position_));
    warn_topic("cf_ctrl_target_pos", age_sec(t, t_last_cf_ctrl_target_pos_));
    warn_topic("stateEstimate_velocity", age_sec(t, t_last_state_estimate_vel_));
    warn_topic("stateEstimate_acc", age_sec(t, t_last_state_estimate_acc_));
    warn_topic("gyro_feedback", age_sec(t, t_last_gyro_feedback_));
    warn_topic("vel_des", age_sec(t, t_last_vel_des_));
    warn_topic("att_des", age_sec(t, t_last_att_des_));
    warn_topic("rate_des", age_sec(t, t_last_rate_des_));
    warn_topic("cf_motor_force", age_sec(t, t_last_cf_motor_force_));
    warn_topic("cf_motor_force_scaled", age_sec(t, t_last_cf_motor_force_scaled_));
    warn_topic("cf_body_input_force", age_sec(t, t_last_cf_body_input_force_));
    warn_topic("cf_world_force", age_sec(t, t_last_cf_world_force_));
    warn_topic("cf_F_input_scaled", age_sec(t, t_last_cf_F_input_scaled_));
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
    t_last_pose_ = now_sec();
  }

  void statusCallback(const crazyflie_interfaces::msg::Status::SharedPtr msg)
  {
    status_batt_v_ = msg->battery_voltage;
    t_last_status_ = now_sec();
  }

  void cfVoltageCallback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg)
  {
    if (msg->values.size() >= 2) {
      cf_voltage_[0] = msg->values[0];
      cf_voltage_[1] = msg->values[1];
      t_last_cf_voltage_ = now_sec();
    }
  }

  void cmdPositionCallback(const crazyflie_interfaces::msg::Position::SharedPtr msg)
  {
    cmd_xyzyaw_[0] = msg->x;
    cmd_xyzyaw_[1] = msg->y;
    cmd_xyzyaw_[2] = msg->z;
    cmd_xyzyaw_[3] = msg->yaw;
    t_last_cmd_position_ = now_sec();
  }

  void stateEstimateVelocityCallback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg)
  {
    if (msg->values.size() >= 3) {
      state_estimate_vel_[0] = msg->values[0];
      state_estimate_vel_[1] = msg->values[1];
      state_estimate_vel_[2] = msg->values[2];
      t_last_state_estimate_vel_ = now_sec();
    }
  }

  void cfCtrlTargetPosCallback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg)
  {
    if (msg->values.size() >= 3) {
      fw_cmd_xyz_[0] = msg->values[0];
      fw_cmd_xyz_[1] = msg->values[1];
      fw_cmd_xyz_[2] = msg->values[2];
      t_last_cf_ctrl_target_pos_ = now_sec();
    }
  }

  void stateEstimateAccCallback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg)
  {
    if (msg->values.size() >= 3) {
      state_estimate_acc_[0] = msg->values[0];
      state_estimate_acc_[1] = msg->values[1];
      state_estimate_acc_[2] = msg->values[2];
      t_last_state_estimate_acc_ = now_sec();
    }
  }

  void gyroFeedbackCallback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg)
  {
    if (msg->values.size() < 3) {
      return;
    }

    const double now = now_sec();
    std::array<double, 3> gyro_now = {msg->values[0], msg->values[1], msg->values[2]};

    if (std::isfinite(t_last_gyro_feedback_) && std::isfinite(prev_gyro_sample_time_)) {
      const double dt = now - prev_gyro_sample_time_;
      if (dt > 1e-6) {
        for (size_t i = 0; i < 3; ++i) {
          ang_acc_feedback_[i] = (gyro_now[i] - prev_gyro_feedback_[i]) / dt;
        }
      }
    }

    gyro_feedback_ = gyro_now;
    prev_gyro_feedback_ = gyro_now;
    prev_gyro_sample_time_ = now;
    t_last_gyro_feedback_ = now;
  }

  void velDesCallback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg)
  {
    if (msg->values.size() >= 3) {
      vel_des_[0] = msg->values[0];
      vel_des_[1] = msg->values[1];
      vel_des_[2] = msg->values[2];
      t_last_vel_des_ = now_sec();
    }
  }

  void attDesCallback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg)
  {
    if (msg->values.size() >= 3) {
      att_des_[0] = msg->values[0];
      att_des_[1] = msg->values[1];
      att_des_[2] = msg->values[2];
      t_last_att_des_ = now_sec();
    }
  }

  void rateDesCallback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg)
  {
    if (msg->values.size() >= 3) {
      rate_des_[0] = msg->values[0];
      rate_des_[1] = msg->values[1];
      rate_des_[2] = msg->values[2];
      t_last_rate_des_ = now_sec();
    }
  }

  void cfMotorForceCallback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg)
  {
    if (msg->values.size() >= 4) {
      for (size_t i = 0; i < 4; ++i) {
        motor_force_[i] = msg->values[i];
      }
      t_last_cf_motor_force_ = now_sec();
    }
  }

  void cfMotorForceScaledCallback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg)
  {
    if (msg->values.size() >= 4) {
      for (size_t i = 0; i < 4; ++i) {
        motor_force_scaled_[i] = msg->values[i];
      }
      t_last_cf_motor_force_scaled_ = now_sec();
    }
  }

  void cfBodyInputForceCallback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg)
  {
    if (msg->values.size() >= 3) {
      cf_body_input_force_[0] = msg->values[0];
      cf_body_input_force_[1] = msg->values[1];
      cf_body_input_force_[2] = msg->values[2];
      t_last_cf_body_input_force_ = now_sec();
    }
  }

  void cfWorldForceCallback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg)
  {
    if (msg->values.size() >= 3) {
      cf_world_force_[0] = msg->values[0];
      cf_world_force_[1] = msg->values[1];
      cf_world_force_[2] = msg->values[2];
      t_last_cf_world_force_ = now_sec();
    }
  }

  void cfFInputScaledCallback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg)
  {
    if (msg->values.size() >= 3) {
      cf_F_input_scaled_[0] = msg->values[0];
      cf_F_input_scaled_[1] = msg->values[1];
      cf_F_input_scaled_[2] = msg->values[2];
      t_last_cf_F_input_scaled_ = now_sec();
    }
  }

  double now_sec() { return get_clock()->now().seconds(); }

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr data_pub_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_;
  rclcpp::Subscription<crazyflie_interfaces::msg::Status>::SharedPtr sub_status_;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr sub_cf_voltage_;
  rclcpp::Subscription<crazyflie_interfaces::msg::Position>::SharedPtr sub_cmd_position_;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr sub_cf_ctrl_target_pos_;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr sub_state_estimate_velocity_;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr sub_state_estimate_acc_;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr sub_gyro_feedback_;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr sub_vel_des_;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr sub_att_des_;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr sub_rate_des_;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr sub_cf_motor_force_;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr sub_cf_motor_force_scaled_;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr sub_cf_body_input_force_;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr sub_cf_world_force_;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr sub_cf_F_input_scaled_;

  std::string csv_dir_;
  std::string csv_path_;
  std::ofstream csv_;
  uint64_t csv_line_count_{0};
  int flush_every_n_{200};
  bool have_published_once_{false};

  std::string publish_topic_;
  std::string cf_ns_;
  double loop_hz_{100.0};
  double stale_warn_sec_{0.5};
  double stale_fail_sec_{2.0};

  std::array<double, 3> pose_xyz_ = {qnan(), qnan(), qnan()};
  std::array<double, 3> pose_rpy_ = {qnan(), qnan(), qnan()};
  double status_batt_v_ = qnan();
  std::array<double, 2> cf_voltage_ = {qnan(), qnan()};
  std::array<double, 4> cmd_xyzyaw_ = {qnan(), qnan(), qnan(), qnan()};
  std::array<double, 3> fw_cmd_xyz_ = {qnan(), qnan(), qnan()};
  std::array<double, 3> state_estimate_vel_ = {qnan(), qnan(), qnan()};
  std::array<double, 3> state_estimate_acc_ = {qnan(), qnan(), qnan()};
  std::array<double, 3> gyro_feedback_ = {qnan(), qnan(), qnan()};
  std::array<double, 3> ang_acc_feedback_ = {qnan(), qnan(), qnan()};
  std::array<double, 3> prev_gyro_feedback_ = {qnan(), qnan(), qnan()};
  std::array<double, 3> vel_des_ = {qnan(), qnan(), qnan()};
  std::array<double, 3> att_des_ = {qnan(), qnan(), qnan()};
  std::array<double, 3> rate_des_ = {qnan(), qnan(), qnan()};
  std::array<double, 4> motor_force_ = {qnan(), qnan(), qnan(), qnan()};
  std::array<double, 4> motor_force_scaled_ = {qnan(), qnan(), qnan(), qnan()};
  std::array<double, 3> cf_body_input_force_ = {qnan(), qnan(), qnan()};
  std::array<double, 3> cf_world_force_ = {qnan(), qnan(), qnan()};
  std::array<double, 3> cf_F_input_scaled_ = {qnan(), qnan(), qnan()};
  double zero_bias_count_ = qnan();

  double t_last_pose_ = qnan();
  double t_last_status_ = qnan();
  double t_last_cf_voltage_ = qnan();
  double t_last_cmd_position_ = qnan();
  double t_last_cf_ctrl_target_pos_ = qnan();
  double t_last_state_estimate_vel_ = qnan();
  double t_last_state_estimate_acc_ = qnan();
  double t_last_gyro_feedback_ = qnan();
  double prev_gyro_sample_time_ = qnan();
  double t_last_vel_des_ = qnan();
  double t_last_att_des_ = qnan();
  double t_last_rate_des_ = qnan();
  double t_last_cf_motor_force_ = qnan();
  double t_last_cf_motor_force_scaled_ = qnan();
  double t_last_cf_body_input_force_ = qnan();
  double t_last_cf_world_force_ = qnan();
  double t_last_cf_F_input_scaled_ = qnan();
  double t_last_cf_zero_bias_dbg_ = qnan();
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DataLoggingNode>();
  rclcpp::Rate rate(node->loop_hz());

  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    node->loopOnce();
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
