#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <crazyflie_interfaces/msg/log_data_generic.hpp>
#include <crazyflie_interfaces/msg/position.hpp>
#include <crazyflie_interfaces/msg/position_control.hpp>
#include <crazyflie_interfaces/msg/status.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>

#include <ncurses.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <deque>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

using namespace std::chrono_literals;

class CommandPublisher : public rclcpp::Node
{
public:
  explicit CommandPublisher(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("command_publisher", options)
  {
    cf_position_pub_ =
      this->create_publisher<crazyflie_interfaces::msg::Position>(
        "cf2/cmd_position",
        rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

    position_control_pub_ =
      this->create_publisher<crazyflie_interfaces::msg::PositionControl>(
        "cf2/cmd_position_control",
        rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

    key_pub_ = this->create_publisher<std_msgs::msg::String>(
      "keyboard_input", 10);

    status_sub_ = this->create_subscription<crazyflie_interfaces::msg::Status>(
      "cf2/status", 10,
      std::bind(&CommandPublisher::statusCallback, this, std::placeholders::_1));
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "cf2/pose", 10,
      std::bind(&CommandPublisher::poseCallback, this, std::placeholders::_1));
    fw_cmd_sub_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      "cf2/cf_ctrl_target_pos", 10,
      std::bind(&CommandPublisher::fwCmdPositionCallback, this, std::placeholders::_1));
    mob_force_sub_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      "cf2/cf_Fext_MOB", 10,
      std::bind(&CommandPublisher::mobForceCallback, this, std::placeholders::_1));

    position_tick_ = declareVector3Parameter("position_tick", {0.1, 0.1, 0.2});
    velocity_tick_ = declareVector3Parameter("velocity_tick", {0.1, 0.1, 0.1});
    yaw_tick_deg_ = this->declare_parameter<double>("yaw_tick_deg", 2.0);
    force_delta_ = this->declare_parameter<double>("force_tick", 0.01);
    command_frame_ = this->declare_parameter<std::string>("command_frame", loadSharedCommandFrameDefault());
    end_effector_offset_ = declareVector3Parameter("end_effector_offset", loadSharedEndEffectorOffsetDefault());

    trajectory_label_none_ = this->declare_parameter<std::string>("trajectory_none_label", "none");
    trajectory_label_1_ = this->declare_parameter<std::string>("trajectory_1_label", "trajectory_run");

    cmd_xyz_yaw_.fill(0.0);
    force_des_ = 0.0;
    current_mode_ = crazyflie_interfaces::msg::PositionControl::MODE_POSITION;
    current_command_reference_ = parseCommandReference(command_frame_);
    current_trajectory_mode_ = crazyflie_interfaces::msg::PositionControl::TRAJECTORY_NONE;
    mob_force_.fill(std::numeric_limits<double>::quiet_NaN());
    latest_battery_voltage_ = std::numeric_limits<double>::quiet_NaN();
    displayed_battery_voltage_ = std::numeric_limits<double>::quiet_NaN();
    status_msg_ = "ready";
    last_battery_display_update_ = std::chrono::steady_clock::now();
    has_latest_pose_ = false;
    has_latest_fw_cmd_ = false;
    command_initialized_ = true;

    last_inputs_.clear();
    for (size_t i = 0; i < HISTORY_LEN; ++i) {
      last_inputs_.push_back("-");
    }

    initscr();
    cbreak();
    noecho();
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);

    drawLayout();
    publishPositionControl();

    RCLCPP_INFO(
      this->get_logger(),
      "command_publisher started. position_tick=(%.3f, %.3f, %.3f) velocity_tick=(%.3f, %.3f, %.3f)",
      position_tick_[0], position_tick_[1], position_tick_[2],
      velocity_tick_[0], velocity_tick_[1], velocity_tick_[2]);

    timer_ = this->create_wall_timer(
      50ms, std::bind(&CommandPublisher::timerCallback, this));
  }

  ~CommandPublisher() override
  {
    endwin();
  }

private:
  static constexpr int ROW_USAGE_HEADER = 0;
  static constexpr int ROW_USAGE_1 = 2;
  static constexpr int ROW_USAGE_2 = 3;
  static constexpr int ROW_USAGE_3 = 4;

  static constexpr int ROW_STATUS_HEADER = 6;
  static constexpr int ROW_STATUS_MODE = 8;
  static constexpr int ROW_STATUS_TRAJ = 9;
  static constexpr int ROW_STATUS_FORCE = 10;
  static constexpr int ROW_STATUS_MOB = 11;
  static constexpr int ROW_STATUS_BATTERY = 12;
  static constexpr int ROW_STATUS_MSG = 14;

  static constexpr int ROW_CMD_HEADER = 16;
  static constexpr int ROW_CMD_LINE1 = 18;
  static constexpr int ROW_CMD_LINE2 = 19;
  static constexpr int ROW_CMD_HIST_HDR = 21;
  static constexpr int ROW_CMD_HIST_0 = 22;
  static constexpr size_t HISTORY_LEN = 5;

  std::array<double, 3> declareVector3Parameter(
    const std::string & name,
    const std::vector<double> & defaultValue)
  {
    const auto values = this->declare_parameter<std::vector<double>>(name, defaultValue);
    std::array<double, 3> result{{0.0, 0.0, 0.0}};

    if (values.size() != 3) {
      RCLCPP_WARN(
        this->get_logger(),
        "Parameter '%s' must contain exactly 3 values. Using zeros.",
        name.c_str());
      return result;
    }

    for (size_t i = 0; i < 3; ++i) {
      result[i] = values[i];
    }

    return result;
  }

  std::string loadSharedCommandFrameDefault() const
  {
    const auto shared_yaml =
      ament_index_cpp::get_package_share_directory("crazyflie") + "/config/su_params.yaml";
    std::ifstream file(shared_yaml);
    if (!file.is_open()) {
      return "end_effector";
    }

    std::string line;
    while (std::getline(file, line)) {
      if (line.find("command_reference:") == std::string::npos) {
        continue;
      }

      const auto colon = line.find(':');
      if (colon == std::string::npos || colon + 1 >= line.size()) {
        break;
      }

      std::string value = line.substr(colon + 1);
      value.erase(0, value.find_first_not_of(" \t"));
      const auto last = value.find_last_not_of(" \t\r\n");
      if (last != std::string::npos) {
        value.erase(last + 1);
      } else {
        value.clear();
      }

      return value == "drone" ? "drone" : "end_effector";
    }

    return "end_effector";
  }

  std::vector<double> loadSharedEndEffectorOffsetDefault() const
  {
    const auto shared_yaml =
      ament_index_cpp::get_package_share_directory("crazyflie") + "/config/su_params.yaml";
    std::ifstream file(shared_yaml);
    if (!file.is_open()) {
      return {0.1, 0.0, 0.04};
    }

    bool insideSuWrench = false;
    std::array<double, 3> offset{{0.1, 0.0, 0.04}};
    std::array<bool, 3> found{{false, false, false}};
    std::string line;
    while (std::getline(file, line)) {
      if (!insideSuWrench) {
        if (line.find("su_wrench:") != std::string::npos) {
          insideSuWrench = true;
        }
        continue;
      }

      if (line.rfind("      su_", 0) == 0 && line.find("su_wrench:") == std::string::npos) {
        break;
      }

      auto parseValue = [&](const std::string & key, double & dst, bool & ok) {
        if (line.find(key) == std::string::npos) {
          return;
        }
        const auto colon = line.find(':');
        if (colon == std::string::npos || colon + 1 >= line.size()) {
          return;
        }
        try {
          dst = std::stod(line.substr(colon + 1));
          ok = true;
        } catch (...) {
        }
      };

      parseValue("rOffX:", offset[0], found[0]);
      parseValue("rOffY:", offset[1], found[1]);
      parseValue("rOffZ:", offset[2], found[2]);
      if (found[0] && found[1] && found[2]) {
        return {offset[0], offset[1], offset[2]};
      }
    }

    return {offset[0], offset[1], offset[2]};
  }

  void timerCallback()
  {
    int ch = 0;
    while ((ch = getch()) != ERR) {
      handleKey(static_cast<char>(ch));
    }

    updateDisplayedBatteryVoltage();
    publishPositionCmd();
    drawStatusBlock();
    drawCommandBlock();
  }

  void handleKey(char c)
  {
    if (c == 'w')       { onPositiveX(); }
    else if (c == 's')  { onNegativeX(); }
    else if (c == 'a')  { onPositiveY(); }
    else if (c == 'd')  { onNegativeY(); }
    else if (c == 'e')  { onPositiveZ(); }
    else if (c == 'q')  { onNegativeZ(); }
    else if (c == 'z')  { cmd_xyz_yaw_[3] += yaw_tick_deg_; pushInputHistory("z : yaw += tick"); }
    else if (c == 'c')  { cmd_xyz_yaw_[3] -= yaw_tick_deg_; pushInputHistory("c : yaw -= tick"); }
    else if (c == 'x')  { resetActiveCommand(); }
    else if (c == 'i')  { setPositionMode(crazyflie_interfaces::msg::PositionControl::MODE_VELOCITY); }
    else if (c == 'u')  { setPositionMode(crazyflie_interfaces::msg::PositionControl::MODE_POSITION); }
    else if (c == 'f')  { publishKeyboardTrigger('f'); }
    else if (c == 'g')  { status_msg_ = "runtime command reference switching disabled"; pushInputHistory("g : reference switching disabled"); }
    else if (c == 'n')  { setTrajectoryMode(crazyflie_interfaces::msg::PositionControl::TRAJECTORY_NONE); }
    else if (c == 'm')  { setTrajectoryMode(crazyflie_interfaces::msg::PositionControl::TRAJECTORY_1); }
    else if (c == 'j')  { force_des_ += force_delta_; publishPositionControl(); updateForceStatus(); pushInputHistory("j : force += tick"); }
    else if (c == 'k')  { force_des_ -= force_delta_; publishPositionControl(); updateForceStatus(); pushInputHistory("k : force -= tick"); }
    else if (c == 'l')  { force_des_ = 0.0; publishPositionControl(); updateForceStatus(); pushInputHistory("l : force reset"); }
    else if (c == 'o' || c == 'p') {
      publishKeyboardTrigger(c);
    } else if (c == 't') {
      pushInputHistory("t : quit");
      status_msg_ = "exit key pressed";
      rclcpp::shutdown();
    }
  }

  bool isContactFrameVelocityMode() const
  {
    return current_mode_ == crazyflie_interfaces::msg::PositionControl::MODE_VELOCITY;
  }

  bool usesVelocityCommands() const
  {
    return true;
  }

  uint8_t parseCommandReference(const std::string & frame) const
  {
    return frame == "drone" ?
      crazyflie_interfaces::msg::PositionControl::REFERENCE_DRONE :
      crazyflie_interfaces::msg::PositionControl::REFERENCE_END_EFFECTOR;
  }

  bool isEndEffectorReference() const
  {
    return current_command_reference_ == crazyflie_interfaces::msg::PositionControl::REFERENCE_END_EFFECTOR;
  }

  void onPositiveX()
  {
    if (usesVelocityCommands()) {
      cmd_xyz_yaw_[0] += velocity_tick_[0];
      pushInputHistory("w : vx += tick");
    } else {
      cmd_xyz_yaw_[0] += position_tick_[0];
      pushInputHistory("w : x += tick");
    }
  }

  void onNegativeX()
  {
    if (usesVelocityCommands()) {
      cmd_xyz_yaw_[0] -= velocity_tick_[0];
      pushInputHistory("s : vx -= tick");
    } else {
      cmd_xyz_yaw_[0] -= position_tick_[0];
      pushInputHistory("s : x -= tick");
    }
  }

  void onPositiveY()
  {
    if (usesVelocityCommands()) {
      cmd_xyz_yaw_[1] += velocity_tick_[1];
      pushInputHistory("a : vy += tick");
    } else {
      cmd_xyz_yaw_[1] += position_tick_[1];
      pushInputHistory("a : y += tick");
    }
  }

  void onNegativeY()
  {
    if (usesVelocityCommands()) {
      cmd_xyz_yaw_[1] -= velocity_tick_[1];
      pushInputHistory("d : vy -= tick");
    } else {
      cmd_xyz_yaw_[1] -= position_tick_[1];
      pushInputHistory("d : y -= tick");
    }
  }

  void onPositiveZ()
  {
    if (usesVelocityCommands()) {
      cmd_xyz_yaw_[2] += velocity_tick_[2];
      pushInputHistory("e : vz += tick");
    } else {
      cmd_xyz_yaw_[2] += position_tick_[2];
      pushInputHistory("e : z += tick");
    }
  }

  void onNegativeZ()
  {
    if (usesVelocityCommands()) {
      cmd_xyz_yaw_[2] -= velocity_tick_[2];
      pushInputHistory("q : vz -= tick");
    } else {
      cmd_xyz_yaw_[2] -= position_tick_[2];
      pushInputHistory("q : z -= tick");
    }
  }

  void setPositionMode(uint8_t mode)
  {
    if (mode == current_mode_) {
      status_msg_ = isContactFrameVelocityMode() ?
        "already in CONTACT-FRAME velocity mode" :
        "already in WORLD-FRAME velocity mode";
      return;
    }
    current_mode_ = mode;
    velocity_mode_command_ready_time_ = std::chrono::steady_clock::time_point{};
    publishPositionControl();
    if (isContactFrameVelocityMode()) {
      status_msg_ = "contact-frame velocity mode enabled";
      pushInputHistory("i : contact-frame velocity ON");
    } else {
      status_msg_ = "world-frame velocity mode enabled";
      pushInputHistory("u : contact-frame velocity OFF");
    }
  }

  void setCommandReference(uint8_t commandReference)
  {
    if (commandReference == current_command_reference_) {
      status_msg_ = isEndEffectorReference() ? "already in END_EFFECTOR reference" : "already in DRONE reference";
      return;
    }

    if (has_latest_pose_) {
      const auto latest_reference_pose =
        commandReference == crazyflie_interfaces::msg::PositionControl::REFERENCE_END_EFFECTOR ?
          latestPoseInReferenceFrame(crazyflie_interfaces::msg::PositionControl::REFERENCE_END_EFFECTOR) :
          latestPoseInReferenceFrame(crazyflie_interfaces::msg::PositionControl::REFERENCE_DRONE);
      cmd_xyz_yaw_[0] = latest_reference_pose[0];
      cmd_xyz_yaw_[1] = latest_reference_pose[1];
      cmd_xyz_yaw_[2] = latest_reference_pose[2];
      cmd_xyz_yaw_[3] = latest_pose_xyz_yaw_[3];
    }

    current_command_reference_ = commandReference;
    command_frame_ = isEndEffectorReference() ? "end_effector" : "drone";
    publishPositionControl();
    if (isEndEffectorReference()) {
      status_msg_ = "command reference set to END_EFFECTOR";
      pushInputHistory("g : command ref -> end_effector");
    } else {
      status_msg_ = "command reference set to DRONE";
      pushInputHistory("f : command ref -> drone");
    }
  }

  void setTrajectoryMode(uint8_t trajectoryMode)
  {
    current_trajectory_mode_ = trajectoryMode;
    publishPositionControl();

    if (trajectoryMode == crazyflie_interfaces::msg::PositionControl::TRAJECTORY_NONE) {
      status_msg_ = "trajectory stopped";
      pushInputHistory("n : trajectory stop");
    } else if (trajectoryMode == crazyflie_interfaces::msg::PositionControl::TRAJECTORY_1) {
      status_msg_ = "trajectory running";
      pushInputHistory("m : trajectory run");
    } else {
      status_msg_ = "trajectory running";
      pushInputHistory("trajectory run");
    }
  }

  void resetActiveCommand()
  {
    if (usesVelocityCommands()) {
      cmd_xyz_yaw_[0] = 0.0;
      cmd_xyz_yaw_[1] = 0.0;
      cmd_xyz_yaw_[2] = 0.0;
      status_msg_ = "velocity command reset to zero";
      pushInputHistory("x : zero velocity cmd");
    } else {
      if (has_latest_pose_) {
        const auto latest_reference_pose = latestPoseInActiveReferenceFrame();
        cmd_xyz_yaw_[0] = latest_reference_pose[0];
        cmd_xyz_yaw_[1] = latest_reference_pose[1];
        cmd_xyz_yaw_[2] = latest_reference_pose[2];
        cmd_xyz_yaw_[3] = latest_pose_xyz_yaw_[3];
        status_msg_ = "position command aligned to latest pose";
        pushInputHistory("x : hold current pose");
      } else {
        cmd_xyz_yaw_[0] = 0.0;
        cmd_xyz_yaw_[1] = 0.0;
        cmd_xyz_yaw_[2] = 0.0;
        status_msg_ = "position command reset to zero";
        pushInputHistory("x : reset position cmd");
      }
    }
  }

  void publishPositionCmd()
  {
    if (!command_initialized_) {
      return;
    }

    if (usesVelocityCommands() &&
        velocity_mode_command_ready_time_.time_since_epoch().count() > 0 &&
        std::chrono::steady_clock::now() < velocity_mode_command_ready_time_) {
      return;
    }

    crazyflie_interfaces::msg::Position msg;
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "world";
    msg.x = static_cast<float>(cmd_xyz_yaw_[0]);
    msg.y = static_cast<float>(cmd_xyz_yaw_[1]);
    msg.z = static_cast<float>(cmd_xyz_yaw_[2]);
    msg.yaw = static_cast<float>(cmd_xyz_yaw_[3]);
    cf_position_pub_->publish(msg);
  }

  std::array<double, 3> rotateOffsetByYawDeg(double yaw_deg) const
  {
    const double yaw_rad = yaw_deg * M_PI / 180.0;
    const double c = std::cos(yaw_rad);
    const double s = std::sin(yaw_rad);

    return {{
      c * end_effector_offset_[0] - s * end_effector_offset_[1],
      s * end_effector_offset_[0] + c * end_effector_offset_[1],
      end_effector_offset_[2]
    }};
  }

  std::array<double, 3> latestPoseInReferenceFrame(uint8_t commandReference) const
  {
    std::array<double, 3> pose{{
      latest_pose_xyz_yaw_[0],
      latest_pose_xyz_yaw_[1],
      latest_pose_xyz_yaw_[2]
    }};

    if (commandReference != crazyflie_interfaces::msg::PositionControl::REFERENCE_END_EFFECTOR) {
      return pose;
    }

    const auto rotated_offset = rotateOffsetByYawDeg(latest_pose_xyz_yaw_[3]);
    pose[0] += rotated_offset[0];
    pose[1] += rotated_offset[1];
    pose[2] += rotated_offset[2];
    return pose;
  }

  std::array<double, 3> latestPoseInActiveReferenceFrame() const
  {
    return latestPoseInReferenceFrame(current_command_reference_);
  }

  std::array<double, 3> latestFwCommandInActiveReferenceFrame() const
  {
    std::array<double, 3> command{{
      latest_fw_cmd_xyz_[0],
      latest_fw_cmd_xyz_[1],
      latest_fw_cmd_xyz_[2]
    }};

    if (!isEndEffectorReference()) {
      return command;
    }

    const double yaw_deg = has_latest_pose_ ? latest_pose_xyz_yaw_[3] : cmd_xyz_yaw_[3];
    const auto rotated_offset = rotateOffsetByYawDeg(yaw_deg);
    command[0] += rotated_offset[0];
    command[1] += rotated_offset[1];
    command[2] += rotated_offset[2];
    return command;
  }

  void publishPositionControl()
  {
    if (!position_control_pub_) {
      return;
    }

    crazyflie_interfaces::msg::PositionControl msg;
    msg.header.stamp = this->get_clock()->now();
    msg.position_mode = current_mode_;
    msg.command_reference = current_command_reference_;
    msg.trajectory_mode = current_trajectory_mode_;
    msg.force_desired = static_cast<float>(force_des_);
    position_control_pub_->publish(msg);
  }

  void updateForceStatus()
  {
    char buf[128];
    snprintf(buf, sizeof(buf), "set force_desired = %.3f (via PositionControl)", force_des_);
    status_msg_ = buf;
  }

  void publishKeyboardTrigger(char key)
  {
    if (!key_pub_) {
      status_msg_ = "key_pub not ready";
      return;
    }

    std_msgs::msg::String msg;
    msg.data = std::string(1, key);
    key_pub_->publish(msg);

    if (key == 'o') {
      status_msg_ = "published 'o' to keyboard_input (ARM)";
      pushInputHistory("o : ARM (keyboard_input)");
    } else if (key == 'p') {
      status_msg_ = "published 'p' to keyboard_input (DISARM)";
      pushInputHistory("p : DISARM (keyboard_input)");
    } else if (key == 'f') {
      status_msg_ = "published 'f' to keyboard_input (hover CoM calibration, configured mass retained)";
      pushInputHistory("f : hover CoM calibration (configured mass retained)");
    } else {
      status_msg_ = "published keyboard trigger";
      pushInputHistory("keyboard trigger published");
    }
  }

  void statusCallback(const crazyflie_interfaces::msg::Status::SharedPtr msg)
  {
    if (msg) {
      latest_battery_voltage_ = msg->battery_voltage;
    }
  }

  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (!msg) {
      return;
    }

    const auto & q = msg->pose.orientation;
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    const double yaw_deg = std::atan2(siny_cosp, cosy_cosp) * 180.0 / M_PI;

    latest_pose_xyz_yaw_[0] = msg->pose.position.x;
    latest_pose_xyz_yaw_[1] = msg->pose.position.y;
    latest_pose_xyz_yaw_[2] = msg->pose.position.z;
    latest_pose_xyz_yaw_[3] = yaw_deg;
    has_latest_pose_ = true;

  }

  void fwCmdPositionCallback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg)
  {
    if (!msg || msg->values.size() < 3) {
      return;
    }

    latest_fw_cmd_xyz_[0] = msg->values[0];
    latest_fw_cmd_xyz_[1] = msg->values[1];
    latest_fw_cmd_xyz_[2] = msg->values[2];
    has_latest_fw_cmd_ = true;
  }

  void mobForceCallback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg)
  {
    if (!msg || msg->values.size() < 3) {
      return;
    }

    mob_force_[0] = msg->values[0];
    mob_force_[1] = msg->values[1];
    mob_force_[2] = msg->values[2];
  }

  void updateDisplayedBatteryVoltage()
  {
    const auto now = std::chrono::steady_clock::now();
    if (now - last_battery_display_update_ < 1s) {
      return;
    }

    last_battery_display_update_ = now;
    displayed_battery_voltage_ = latest_battery_voltage_;
  }

  void pushInputHistory(const std::string & entry)
  {
    last_inputs_.push_front(entry);
    while (last_inputs_.size() > HISTORY_LEN) {
      last_inputs_.pop_back();
    }
    while (last_inputs_.size() < HISTORY_LEN) {
      last_inputs_.push_back("-");
    }
  }

  std::string trajectoryLabel(uint8_t mode) const
  {
    if (mode == crazyflie_interfaces::msg::PositionControl::TRAJECTORY_1) {
      return trajectory_label_1_;
    }
    return trajectory_label_none_;
  }

  void drawSepLine(int row, const char * title)
  {
    move(row, 0);
    clrtoeol();
    printw("========================%s========================", title);
  }

  void drawLayout()
  {
    clear();

    drawSepLine(ROW_USAGE_HEADER, "usage");
    mvprintw(ROW_USAGE_1, 0, "velocity: w/s(x), a/d(y), e/q(z), z/c(yaw), x(zero vel), i(contact on)");
    mvprintw(ROW_USAGE_2, 0, "velocity: w/s/a/d/e/q(v), u(contact off), n(stop), m(run)");
    mvprintw(ROW_USAGE_3, 0, "force/cal: j/k/l (cmd_fx), f(hover CoM, keep mass), o/p arm/disarm, t quit");

    drawSepLine(ROW_STATUS_HEADER, "status");
    mvprintw(ROW_STATUS_MODE, 0, "mode: ");
    mvprintw(ROW_STATUS_TRAJ, 0, "trajectory: ");
    mvprintw(ROW_STATUS_FORCE, 0, "force command: ");
    mvprintw(ROW_STATUS_MOB, 0, "MOB force: ");
    mvprintw(ROW_STATUS_BATTERY, 0, "battery voltage: ");
    mvprintw(ROW_STATUS_MSG, 0, "status: ");

    drawSepLine(ROW_CMD_HEADER, "Position Command, Now");
    mvprintw(ROW_CMD_LINE1, 0, "cmd x/y/z = 0.000 / 0.000 / 0.000");
    mvprintw(ROW_CMD_LINE2, 0, "yaw = 0.0 deg");
    mvprintw(ROW_CMD_HIST_HDR, 0, "last inputs (recent 5):");
    for (size_t i = 0; i < HISTORY_LEN; ++i) {
      mvprintw(ROW_CMD_HIST_0 + static_cast<int>(i), 0, "  %zu) -", i + 1);
    }

    refresh();
  }

  void drawStatusBlock()
  {
    move(ROW_STATUS_MODE, 0);
    clrtoeol();
    printw(
      "mode: %s, frame: %s, pos_tick=(%.2f %.2f %.2f), vel_tick=(%.2f %.2f %.2f)",
      isContactFrameVelocityMode() ? "VEL_CONTACT" : "VEL_WORLD",
      command_frame_.c_str(),
      position_tick_[0], position_tick_[1], position_tick_[2],
      velocity_tick_[0], velocity_tick_[1], velocity_tick_[2]);

    move(ROW_STATUS_TRAJ, 0);
    clrtoeol();
    printw(
      "trajectory: active=%s, n=%s, m=%s",
      trajectoryLabel(current_trajectory_mode_).c_str(),
      trajectory_label_none_.c_str(),
      trajectory_label_1_.c_str());

    move(ROW_STATUS_FORCE, 0);
    clrtoeol();
    printw("force command: %.3f", force_des_);

    move(ROW_STATUS_MOB, 0);
    clrtoeol();
    if (std::isfinite(mob_force_[0]) && std::isfinite(mob_force_[1]) && std::isfinite(mob_force_[2])) {
      const double mob_norm = std::sqrt(
        mob_force_[0] * mob_force_[0] +
        mob_force_[1] * mob_force_[1] +
        mob_force_[2] * mob_force_[2]);
      printw(
        "MOB force: x=%.3f, y=%.3f, z=%.3f [N], norm=%.3f",
        mob_force_[0], mob_force_[1], mob_force_[2], mob_norm);
    } else {
      printw("MOB force: waiting for cf2/cf_Fext_MOB");
    }

    move(ROW_STATUS_BATTERY, 0);
    clrtoeol();
    if (std::isfinite(displayed_battery_voltage_)) {
      printw("battery voltage: %.2f V (1 Hz)", displayed_battery_voltage_);
    } else {
      printw("battery voltage: waiting for cf2/status");
    }

    move(ROW_STATUS_MSG, 0);
    clrtoeol();
    printw("status: %s", status_msg_.c_str());

    refresh();
  }

  void drawCommandBlock()
  {
    move(ROW_CMD_LINE1, 0);
    clrtoeol();
    printw(
      "velocity cmd xyz = %.3f , %.3f , %.3f (reference=%s)",
      cmd_xyz_yaw_[0], cmd_xyz_yaw_[1], cmd_xyz_yaw_[2], command_frame_.c_str());

    move(ROW_CMD_LINE2, 0);
    clrtoeol();
    if (has_latest_pose_) {
      printw(
        "yaw = %.1f deg, latest pose = %.3f , %.3f , %.3f , yaw %.1f",
        cmd_xyz_yaw_[3],
        latest_pose_xyz_yaw_[0], latest_pose_xyz_yaw_[1], latest_pose_xyz_yaw_[2], latest_pose_xyz_yaw_[3]);
    } else {
      printw("yaw = %.1f deg, latest pose = waiting for cf2/pose", cmd_xyz_yaw_[3]);
    }

    mvprintw(ROW_CMD_HIST_HDR, 0, "last inputs (recent 5):");
    for (size_t i = 0; i < HISTORY_LEN; ++i) {
      move(ROW_CMD_HIST_0 + static_cast<int>(i), 0);
      clrtoeol();
      printw("  %zu) %s", i + 1, last_inputs_[i].c_str());
    }

    refresh();
  }

  rclcpp::Publisher<crazyflie_interfaces::msg::Position>::SharedPtr cf_position_pub_;
  rclcpp::Publisher<crazyflie_interfaces::msg::PositionControl>::SharedPtr position_control_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr key_pub_;
  rclcpp::Subscription<crazyflie_interfaces::msg::Status>::SharedPtr status_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr fw_cmd_sub_;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr mob_force_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::array<double, 4> cmd_xyz_yaw_;
  std::array<double, 4> latest_pose_xyz_yaw_{{0.0, 0.0, 0.0, 0.0}};
  std::array<double, 3> latest_fw_cmd_xyz_{{0.0, 0.0, 0.0}};
  std::array<double, 3> position_tick_;
  std::array<double, 3> velocity_tick_;
  std::array<double, 3> end_effector_offset_;
  std::array<double, 3> mob_force_;
  double yaw_tick_deg_;
  double force_des_;
  double force_delta_;
  double latest_battery_voltage_;
  double displayed_battery_voltage_;
  uint8_t current_mode_;
  uint8_t current_command_reference_;
  uint8_t current_trajectory_mode_;
  bool has_latest_pose_;
  bool has_latest_fw_cmd_;
  bool command_initialized_{false};
  std::string command_frame_;
  std::string trajectory_label_none_;
  std::string trajectory_label_1_;
  std::string status_msg_;
  std::chrono::steady_clock::time_point last_battery_display_update_;
  std::chrono::steady_clock::time_point velocity_mode_command_ready_time_{};
  std::deque<std::string> last_inputs_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;

  bool has_params_file = false;
  for (int i = 1; i < argc; ++i) {
    if (std::string(argv[i]) == "--params-file") {
      has_params_file = true;
      break;
    }
  }

  if (!has_params_file) {
    const auto config_path =
      ament_index_cpp::get_package_share_directory("flying_pen") + "/config/command.yaml";
    options.arguments({"--ros-args", "--params-file", config_path});
  }

  auto node = std::make_shared<CommandPublisher>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
