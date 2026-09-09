#include <memory>
#include <vector>
#include <regex>
#include <fstream>
#include <filesystem>
#include <sstream>
#include <iomanip>
#include <crazyflie_cpp/Crazyflie.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include "std_srvs/srv/empty.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "crazyflie_interfaces/srv/start_trajectory.hpp"
#include "crazyflie_interfaces/srv/takeoff.hpp"
#include "crazyflie_interfaces/srv/land.hpp"
#include "crazyflie_interfaces/srv/go_to.hpp"
#include "crazyflie_interfaces/srv/notify_setpoints_stop.hpp"
#include "crazyflie_interfaces/srv/arm.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "crazyflie_interfaces/srv/upload_trajectory.hpp"
#include "motion_capture_tracking_interfaces/msg/named_pose_array.hpp"
#include "crazyflie_interfaces/msg/full_state.hpp"
#include "crazyflie_interfaces/msg/hover.hpp"
#include "crazyflie_interfaces/msg/position.hpp"
#include "crazyflie_interfaces/msg/position_control.hpp"
#include "crazyflie_interfaces/msg/status.hpp"
#include "crazyflie_interfaces/msg/log_data_generic.hpp"
#include "crazyflie_interfaces/msg/velocity_world.hpp"
#include "crazyflie_interfaces/msg/connection_statistics_array.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

using crazyflie_interfaces::srv::StartTrajectory;
using crazyflie_interfaces::srv::Takeoff;
using crazyflie_interfaces::srv::Land;
using crazyflie_interfaces::srv::GoTo;
using crazyflie_interfaces::srv::UploadTrajectory;
using crazyflie_interfaces::srv::NotifySetpointsStop;
using crazyflie_interfaces::srv::Arm;
using std_srvs::srv::Empty;

using motion_capture_tracking_interfaces::msg::NamedPoseArray;
using crazyflie_interfaces::msg::FullState;

namespace {
constexpr uint8_t kPositionControlTriggerMagic = 0xA5;
constexpr uint8_t kPositionControlTriggerVersion = 0x02;
constexpr uint8_t kHoverCalibrationTriggerMagic = 0xA6;
constexpr uint8_t kHoverCalibrationTriggerVersion = 0x01;

std::string replace_yaml_scalar_line(const std::string& line, double value)
{
  static const std::regex line_re(R"(^(\s*[^:]+:\s*)([^#\n]*)(.*)$)");
  std::smatch match;
  if (!std::regex_match(line, match, line_re)) {
    throw std::runtime_error("Could not parse YAML line: " + line);
  }

  std::ostringstream value_stream;
  value_stream << std::fixed << std::setprecision(6) << value;

  std::string suffix = match[3].str();
  if (!suffix.empty() && suffix.front() == '#') {
    suffix = "  " + suffix;
  }

  return match[1].str() + value_stream.str() + suffix;
}
}

#ifdef ROS_DISTRO_HUMBLE
inline auto get_service_qos() { return rmw_qos_profile_services_default; }
#else
inline auto get_service_qos() { return rclcpp::ServicesQoS(); }
#endif

// Note on logging: we use a single logger with string prefixes
// A better way would be to use named child loggers, but these do not
// report to /rosout in humble, see https://github.com/ros2/rclpy/issues/1131
// Once we do not support humble anymore, consider switching to child loggers

// Helper class to convert crazyflie_cpp logging messages to ROS logging messages
class CrazyflieLogger : public Logger
{
public:
  CrazyflieLogger(rclcpp::Logger logger, const std::string& prefix)
      : Logger()
      , logger_(logger)
      , prefix_(prefix)
  {
  }

  virtual ~CrazyflieLogger() {}

  virtual void info(const std::string &msg)
  {
    RCLCPP_INFO(logger_, "%s %s", prefix_.c_str(), msg.c_str());
  }

  virtual void warning(const std::string &msg)
  {
    RCLCPP_WARN(logger_, "%s %s", prefix_.c_str(),  msg.c_str());
  }

  virtual void error(const std::string &msg)
  {
    RCLCPP_ERROR(logger_, "%s %s", prefix_.c_str(), msg.c_str());
  }
private:
  rclcpp::Logger logger_;
  std::string prefix_;
};

std::set<std::string> extract_names(
    const std::map<std::string, rclcpp::ParameterValue> &parameter_overrides,
    const std::string &pattern)
{
  std::set<std::string> result;
  for (const auto &i : parameter_overrides)
  {
    if (i.first.find(pattern) == 0)
    {
      size_t start = pattern.size() + 1;
      size_t end = i.first.find(".", start);
      result.insert(i.first.substr(start, end - start));
    }
  }
  return result;
}

// ROS wrapper for a single Crazyflie object
class CrazyflieROS
{
private:
  struct logPose {
    float x;
    float y;
    float z;
    int32_t quatCompressed;
  } __attribute__((packed));

  struct logScan {
    uint16_t front;
    uint16_t left;
    uint16_t back;
    uint16_t right;
  } __attribute__((packed));

  struct logStatus {
    // general status
    uint16_t supervisorInfo; // supervisor.info
    // battery related
    // Note that using BQ-deck/Bolt one can actually have two batteries at the same time.
    // vbat refers to the battery directly connected to the CF board and might not reflect
    // the "external" battery on BQ/Bolt builds
    uint16_t vbatMV;  // pm.vbatMV
    uint8_t pmState;  // pm.state
    // radio related
    uint8_t rssi;     // radio.rssi
    uint16_t numRxBc; // radio.numRxBc
    uint16_t numRxUc; // radio.numRxUc
  } __attribute__((packed));

public:
  CrazyflieROS(
    const std::string& link_uri,
    const std::string& cf_type,
    const std::string& name,
    rclcpp::Node* node,
    rclcpp::CallbackGroup::SharedPtr callback_group_cf_cmd,
    rclcpp::CallbackGroup::SharedPtr callback_group_cf_srv,
    const CrazyflieBroadcaster* cfbc,
    bool enable_parameters = true)
    : logger_(node->get_logger())
    , cf_logger_(logger_, "[" + name + "]")
    , cf_(
      link_uri,
      cf_logger_,
      std::bind(&CrazyflieROS::on_console, this, std::placeholders::_1))
    , name_(name)
    , node_(node)
    , tf_broadcaster_(node)
    , last_on_latency_(std::chrono::steady_clock::now())
    , cfbc_(cfbc)
  {
    auto sub_opt_cf_cmd = rclcpp::SubscriptionOptions();
    sub_opt_cf_cmd.callback_group = callback_group_cf_cmd;

    // Services
    auto service_qos = rmw_qos_profile_services_default;


    service_emergency_ = node->create_service<Empty>(name + "/emergency", std::bind(&CrazyflieROS::emergency, this, _1, _2),  get_service_qos(), callback_group_cf_srv);
    service_start_trajectory_ = node->create_service<StartTrajectory>(name + "/start_trajectory", std::bind(&CrazyflieROS::start_trajectory, this, _1, _2), get_service_qos(), callback_group_cf_srv);
    service_takeoff_ = node->create_service<Takeoff>(name + "/takeoff", std::bind(&CrazyflieROS::takeoff, this, _1, _2), get_service_qos(), callback_group_cf_srv);
    service_land_ = node->create_service<Land>(name + "/land", std::bind(&CrazyflieROS::land, this, _1, _2), get_service_qos(), callback_group_cf_srv);
    service_go_to_ = node->create_service<GoTo>(name + "/go_to", std::bind(&CrazyflieROS::go_to, this, _1, _2), get_service_qos(), callback_group_cf_srv);
    service_upload_trajectory_ = node->create_service<UploadTrajectory>(name + "/upload_trajectory", std::bind(&CrazyflieROS::upload_trajectory, this, _1, _2), get_service_qos(), callback_group_cf_srv);
    service_notify_setpoints_stop_ = node->create_service<NotifySetpointsStop>(name + "/notify_setpoints_stop", std::bind(&CrazyflieROS::notify_setpoints_stop, this, _1, _2), get_service_qos(), callback_group_cf_srv);
    service_arm_ = node->create_service<Arm>(name + "/arm", std::bind(&CrazyflieROS::arm, this, _1, _2), get_service_qos(), callback_group_cf_srv);

    // Topics

    subscription_cmd_vel_legacy_ = node->create_subscription<geometry_msgs::msg::Twist>(name + "/cmd_vel_legacy", rclcpp::SystemDefaultsQoS(), std::bind(&CrazyflieROS::cmd_vel_legacy_changed, this, _1), sub_opt_cf_cmd);
    subscription_cmd_full_state_ = node->create_subscription<crazyflie_interfaces::msg::FullState>(name + "/cmd_full_state", rclcpp::SystemDefaultsQoS(), std::bind(&CrazyflieROS::cmd_full_state_changed, this, _1), sub_opt_cf_cmd);
    subscription_cmd_hover_ = node->create_subscription<crazyflie_interfaces::msg::Hover>(name + "/cmd_hover", rclcpp::SystemDefaultsQoS(), std::bind(&CrazyflieROS::cmd_hover_changed, this, _1), sub_opt_cf_cmd);
    subscription_cmd_position_ = node->create_subscription<crazyflie_interfaces::msg::Position>(name + "/cmd_position", rclcpp::SystemDefaultsQoS(), std::bind(&CrazyflieROS::cmd_position_changed, this, _1), sub_opt_cf_cmd);
    subscription_cmd_position_control_ = node->create_subscription<crazyflie_interfaces::msg::PositionControl>(name + "/cmd_position_control", rclcpp::SystemDefaultsQoS(), std::bind(&CrazyflieROS::cmd_position_control_changed, this, _1), sub_opt_cf_cmd);
    subscription_hover_calibration_ = node->create_subscription<std_msgs::msg::Float64MultiArray>(name + "/hover_calibration", rclcpp::SystemDefaultsQoS(), std::bind(&CrazyflieROS::hover_calibration_changed, this, _1), sub_opt_cf_cmd);
    subscription_cmd_velocity_world_ = node->create_subscription<crazyflie_interfaces::msg::VelocityWorld>(name + "/cmd_velocity_world", rclcpp::SystemDefaultsQoS(), std::bind(&CrazyflieROS::cmd_velocity_world_changed, this, _1), sub_opt_cf_cmd);
    publisher_debug_log_filename_tag_ = node->create_publisher<std_msgs::msg::String>(
      "/flying_pen/debug_log_filename_tag", 10);

    publisher_robot_description_ = node->create_publisher<std_msgs::msg::String>(name + "/robot_description",
      rclcpp::QoS(1).transient_local());
    {
      auto msg = std::make_unique<std_msgs::msg::String>();
      auto robot_desc = node->get_parameter("robot_description").get_parameter_value().get<std::string>();
      msg->data = std::regex_replace(robot_desc, std::regex("\\$NAME"), name);
      publisher_robot_description_->publish(std::move(msg));
    }

    // spinning timer
    // used to process all incoming radio messages
    spin_timer_ =
      node->create_wall_timer(
      std::chrono::milliseconds(1),
      std::bind(&CrazyflieROS::spin_once, this), callback_group_cf_srv);

    // link statistics
    warning_freq_ = node->get_parameter("warnings.frequency").get_parameter_value().get<float>();
    max_latency_ = node->get_parameter("warnings.communication.max_unicast_latency").get_parameter_value().get<float>();
    min_ack_rate_ = node->get_parameter("warnings.communication.min_unicast_ack_rate").get_parameter_value().get<float>();
    min_unicast_receive_rate_ = node->get_parameter("warnings.communication.min_unicast_receive_rate").get_parameter_value().get<float>();
    min_broadcast_receive_rate_ = node->get_parameter("warnings.communication.min_broadcast_receive_rate").get_parameter_value().get<float>();
    publish_stats_ = node->get_parameter("warnings.communication.publish_stats").get_parameter_value().get<bool>();
    if (publish_stats_) {
      publisher_connection_stats_ = node->create_publisher<crazyflie_interfaces::msg::ConnectionStatisticsArray>(name + "/connection_statistics", 10);
    }

    if (warning_freq_ >= 0.0) {
      cf_.setLatencyCallback(std::bind(&CrazyflieROS::on_latency, this, std::placeholders::_1));
      link_statistics_timer_ =
        node->create_wall_timer(
        std::chrono::milliseconds((int)(1000.0/warning_freq_)),
        std::bind(&CrazyflieROS::on_link_statistics_timer, this), callback_group_cf_srv);

    }

    auto start = std::chrono::system_clock::now();

    cf_.logReset();

    auto node_parameters_iface = node->get_node_parameters_interface();
    const std::map<std::string, rclcpp::ParameterValue> &parameter_overrides =
        node_parameters_iface->get_parameter_overrides();

    // declares lambda, to be used as local function, which re-declares specified parameters for other nodes to query
    auto declare_param = [&parameter_overrides, node](const std::string& param)
    {
      // rclcpp::ParameterValue value(parameter_overridesparam]);
      node->declare_parameter(param, parameter_overrides.at(param));
    };
    declare_param("robots." + name + ".uri");
    declare_param("robots." + name + ".initial_position");

    // declares a lambda, to be used as local function
    auto update_map = [&parameter_overrides](std::map<std::string, rclcpp::ParameterValue>& map, const std::string& pattern)
    {
      for (const auto &i : parameter_overrides) {
        if (i.first.find(pattern) == 0) {
          size_t start = pattern.size() + 1;
          const auto group_and_name = i.first.substr(start);
          map[group_and_name] = i.second;
        }
      }
    };

    auto update_value = [&parameter_overrides](rclcpp::ParameterValue& value, const std::string& pattern)
    {
      for (const auto &i : parameter_overrides) {
        if (i.first.find(pattern) == 0) {
          value = i.second;
        }
      }
    };

    if (enable_parameters) {
      bool query_all_values_on_connect = node->get_parameter("firmware_params.query_all_values_on_connect").get_parameter_value().get<bool>();

      int numParams = 0;
      RCLCPP_INFO(logger_, "[%s] Requesting parameters...", name_.c_str());
      cf_.requestParamToc(/*forceNoCache*/false, /*requestValues*/query_all_values_on_connect);
      for (auto iter = cf_.paramsBegin(); iter != cf_.paramsEnd(); ++iter) {
        auto entry = *iter;
        std::string paramName = name + ".params." + entry.group + "." + entry.name;
        switch (entry.type)
        {
        case Crazyflie::ParamTypeUint8:
          if (query_all_values_on_connect) {
            node->declare_parameter(paramName, cf_.getParam<uint8_t>(entry.id));
          } else {
            node->declare_parameter(paramName, rclcpp::PARAMETER_INTEGER);
          }
          break;
        case Crazyflie::ParamTypeInt8:
          if (query_all_values_on_connect) {
            node->declare_parameter(paramName, cf_.getParam<int8_t>(entry.id));
          } else {
            node->declare_parameter(paramName, rclcpp::PARAMETER_INTEGER);
          }
          break;
        case Crazyflie::ParamTypeUint16:
          if (query_all_values_on_connect) {
            node->declare_parameter(paramName, cf_.getParam<uint16_t>(entry.id));
          } else {
            node->declare_parameter(paramName, rclcpp::PARAMETER_INTEGER);
          }
          break;
        case Crazyflie::ParamTypeInt16:
          if (query_all_values_on_connect) {
            node->declare_parameter(paramName, cf_.getParam<int16_t>(entry.id));
          } else {
            node->declare_parameter(paramName, rclcpp::PARAMETER_INTEGER);
          }
          break;
        case Crazyflie::ParamTypeUint32:
          if (query_all_values_on_connect) {
            node->declare_parameter<int64_t>(paramName, cf_.getParam<uint32_t>(entry.id));
          } else {
            node->declare_parameter(paramName, rclcpp::PARAMETER_INTEGER);
          }
          break;
        case Crazyflie::ParamTypeInt32:
          if (query_all_values_on_connect) {
            node->declare_parameter(paramName, cf_.getParam<int32_t>(entry.id));
          } else {
            node->declare_parameter(paramName, rclcpp::PARAMETER_INTEGER);
          }
          break;
        case Crazyflie::ParamTypeFloat:
          if (query_all_values_on_connect) {
            node->declare_parameter(paramName, cf_.getParam<float>(entry.id));
          } else {
            node->declare_parameter(paramName, rclcpp::PARAMETER_DOUBLE);
          }
          break;
        default:
          RCLCPP_WARN(logger_, "[%s] Unknown param type for %s/%s", name_.c_str(), entry.group.c_str(), entry.name.c_str());
          break;
        }
        // If there is no such parameter in all, add it
        std::string allParamName = "all.params." + entry.group + "." + entry.name;
        if (!node->has_parameter(allParamName)) {
          if (entry.type == Crazyflie::ParamTypeFloat) {
            node->declare_parameter(allParamName, rclcpp::PARAMETER_DOUBLE);
          } else {
            node->declare_parameter(allParamName, rclcpp::PARAMETER_INTEGER);
          }
        }
        ++numParams;
      }
      auto end1 = std::chrono::system_clock::now();
      std::chrono::duration<double> elapsedSeconds1 = end1 - start;
      RCLCPP_INFO(logger_, "[%s] reqParamTOC: %f s (%d params)", name_.c_str(), elapsedSeconds1.count(), numParams);
      
      // Set parameters as specified in the configuration files, as in the following order
      // 1.) check all/firmware_params
      // 2.) check robot_types/<type_name>/firmware_params
      // 3.) check robots/<robot_name>/firmware_params
      // where the higher order is used if defined on multiple levels.

      // <group>.<name> -> value map
      std::map<std::string, rclcpp::ParameterValue> set_param_map;

      // check global settings/firmware_params
      update_map(set_param_map, "all.firmware_params");
      // check robot_types/<type_name>/firmware_params
      update_map(set_param_map, "robot_types." + cf_type + ".firmware_params");
      // check robots/<robot_name>/firmware_params
      update_map(set_param_map, "robots." + name_ + ".firmware_params");

      const auto configured_mass = set_param_map.find("su_wrench.mass");
      if (configured_mass != set_param_map.end()) {
        if (configured_mass->second.get_type() == rclcpp::PARAMETER_DOUBLE) {
          hover_calibration_mass_ = configured_mass->second.get<double>();
          has_hover_calibration_mass_ = true;
        } else if (configured_mass->second.get_type() == rclcpp::PARAMETER_INTEGER) {
          hover_calibration_mass_ = static_cast<double>(configured_mass->second.get<int64_t>());
          has_hover_calibration_mass_ = true;
        }
      }

      // Update parameters
      for (const auto&i : set_param_map) {
        std::string paramName = name + ".params." + std::regex_replace(i.first, std::regex("\\."), ".");
        change_parameter(rclcpp::Parameter(paramName, i.second));
      }
    }

    // Reference Frame
    {
      rclcpp::ParameterValue reference_frame_value("world");

      // Get logging configuration as specified in the configuration files, as in the following order
      // 1.) check all/reference_frame
      // 2.) check robot_types/<type_name>/reference_frame
      // 3.) check robots/<robot_name>/reference_frame
      // where the higher order is used if defined on multiple levels.
      update_value(reference_frame_value, "all.reference_frame");
      // check robot_types/<type_name>/reference_frame
      update_value(reference_frame_value, "robot_types." + cf_type + ".reference_frame");
      // check robots/<robot_name>/reference_frame
      update_value(reference_frame_value, "robots." + name_ + ".reference_frame");

      // extract reference frame 
      reference_frame_ = reference_frame_value.get<std::string>();
     
      RCLCPP_INFO(logger_, "[%s] ref frame: %s", name_.c_str(), reference_frame_.c_str());
      
    }

    // Logging
    {
      // <group>.<name> -> value map
      std::map<std::string, rclcpp::ParameterValue> log_config_map;

      // Get logging configuration as specified in the configuration files, as in the following order
      // 1.) check all/firmware_logging
      // 2.) check robot_types/<type_name>/firmware_logging
      // 3.) check robots/<robot_name>/firmware_logging
      // where the higher order is used if defined on multiple levels.
      update_map(log_config_map, "all.firmware_logging");
      // check robot_types/<type_name>/firmware_logging
      update_map(log_config_map, "robot_types." + cf_type + ".firmware_logging");
      // check robots/<robot_name>/firmware_logging
      update_map(log_config_map, "robots." + name_ + ".firmware_logging");

      // check if logging is enabled for this drone
      bool logging_enabled = log_config_map["enabled"].get<bool>();
      if (logging_enabled) {
        cf_.requestLogToc(/*forceNoCache*/);

        for (const auto&i : log_config_map) {
          // check if any of the default topics are enabled
          if (i.first.find("default_topics.pose") == 0) {
            int freq = log_config_map["default_topics.pose.frequency"].get<int>();
            RCLCPP_INFO(logger_, "[%s] Logging to /pose at %d Hz", name_.c_str(), freq);

            publisher_pose_ = node->create_publisher<geometry_msgs::msg::PoseStamped>(name + "/pose", 10);

            std::function<void(uint32_t, const logPose*)> cb = std::bind(&CrazyflieROS::on_logging_pose, this, std::placeholders::_1, std::placeholders::_2);

            log_block_pose_.reset(new LogBlock<logPose>(
              &cf_,{
                {"stateEstimate", "x"},
                {"stateEstimate", "y"},
                {"stateEstimate", "z"},
                {"stateEstimateZ", "quat"}
              }, cb));
            log_block_pose_->start(uint8_t(100.0f / (float)freq)); // this is in tens of milliseconds
          }
          else if (i.first.find("default_topics.scan") == 0) {
            int freq = log_config_map["default_topics.scan.frequency"].get<int>();
            RCLCPP_INFO(logger_, "[%s] Logging to /scan at %d Hz", name_.c_str(), freq);

            publisher_scan_ = node->create_publisher<sensor_msgs::msg::LaserScan>(name + "/scan", 10);

            std::function<void(uint32_t, const logScan*)> cb = std::bind(&CrazyflieROS::on_logging_scan, this, std::placeholders::_1, std::placeholders::_2);

            log_block_scan_.reset(new LogBlock<logScan>(
              &cf_,{
                {"range", "front"},
                {"range", "left"},
                {"range", "back"},
                {"range", "right"}
              }, cb));
            log_block_scan_->start(uint8_t(100.0f / (float)freq)); // this is in tens of milliseconds
          }
          else if (i.first.find("default_topics.status") == 0) {
            int freq = log_config_map["default_topics.status.frequency"].get<int>();
            RCLCPP_INFO(logger_, "[%s] Logging to /status at %d Hz", name_.c_str(), freq);

            publisher_status_ = node->create_publisher<crazyflie_interfaces::msg::Status>(name + "/status", 10);

            std::function<void(uint32_t, const logStatus*)> cb = std::bind(&CrazyflieROS::on_logging_status, this, std::placeholders::_1, std::placeholders::_2);

            std::list<std::pair<std::string, std::string> > logvars({
              // general status
              {"supervisor", "info"},
              // battery related
              {"pm", "vbatMV"},
              {"pm", "state"},
              // radio related
              {"radio", "rssi"}
            });

            // check if this firmware version has radio.numRx{Bc,Uc}
            status_has_radio_stats_ = false;
            for (auto iter = cf_.logVariablesBegin(); iter != cf_.logVariablesEnd(); ++iter) {
              auto entry = *iter;
              if (entry.group == "radio" && entry.name == "numRxBc") {
                logvars.push_back({"radio", "numRxBc"});
                logvars.push_back({"radio", "numRxUc"});
                status_has_radio_stats_ = true;
                break;
              }
            }

            // older firmware -> use other 16-bit variables
            if (!status_has_radio_stats_) {
                RCLCPP_WARN(logger_, "[%s] Older firmware. status/num_rx_broadcast and status/num_rx_unicast are set to zero.", name_.c_str());
                logvars.push_back({"pm", "vbatMV"});
                logvars.push_back({"pm", "vbatMV"});
            }

            log_block_status_.reset(new LogBlock<logStatus>(
              &cf_,logvars, cb));
            log_block_status_->start(uint8_t(100.0f / (float)freq)); // this is in tens of milliseconds
          }
          else if (i.first.find("custom_topics") == 0
                   && i.first.rfind(".vars") != std::string::npos) {
            std::string topic_name = i.first.substr(14, i.first.size() - 14 - 5);

            int freq = log_config_map["custom_topics." + topic_name + ".frequency"].get<int>();
            auto vars = log_config_map["custom_topics." + topic_name + ".vars"].get<std::vector<std::string>>();
            
            RCLCPP_INFO(logger_, "[%s] Logging to %s at %d Hz", name_.c_str(), topic_name.c_str(), freq);

            publishers_generic_.emplace_back(node->create_publisher<crazyflie_interfaces::msg::LogDataGeneric>(name + "/" + topic_name, 10));

            std::function<void(uint32_t, const std::vector<float>*, void* userData)> cb = std::bind(
              &CrazyflieROS::on_logging_custom,
              this,
              std::placeholders::_1,
              std::placeholders::_2,
              std::placeholders::_3);

            log_blocks_generic_.emplace_back(new LogBlockGeneric(
              &cf_,
              vars,
              (void*)&publishers_generic_.back(),
              cb));
            log_blocks_generic_.back()->start(uint8_t(100.0f / (float)freq)); // this is in tens of milliseconds
          }
        }
      }
    }

    RCLCPP_INFO(logger_, "[%s] Requesting memories...", name_.c_str());
    cf_.requestMemoryToc();
  }

  void spin_once()
  {
    // process all packets from the receive queue
    cf_.processAllPackets();
  }

  std::string broadcastUri() const
  {
    return cf_.broadcastUri();
  }

  uint8_t id() const
  {
    return cf_.address() & 0xFF;
  }

  const Crazyflie::ParamTocEntry* paramTocEntry(const std::string& group, const std::string& name)
  {
    return cf_.getParamTocEntry(group, name);
  }

  const std::string& name() const
  {
    return name_;
  }

  void change_parameter(const rclcpp::Parameter& p)
  {
    std::string prefix = name_ + ".params.";
    if (p.get_name().find(prefix) != 0) {
      RCLCPP_ERROR(
              logger_,
              "[%s] Incorrect parameter update request for param \"%s\"", name_.c_str(), p.get_name().c_str());
      return;
    }
    size_t pos = p.get_name().find(".", prefix.size());
    std::string group(p.get_name().begin() + prefix.size(), p.get_name().begin() + pos);
    std::string name(p.get_name().begin() + pos + 1, p.get_name().end());

    RCLCPP_INFO(
        logger_,
        "[%s] Update parameter \"%s.%s\" to %s",
        name_.c_str(),
        group.c_str(),
        name.c_str(),
        p.value_to_string().c_str());

    auto entry = cf_.getParamTocEntry(group, name);
    if (entry) {
      switch (entry->type)
      {
      case Crazyflie::ParamTypeUint8:
        cf_.setParam<uint8_t>(entry->id, p.as_int());
        break;
      case Crazyflie::ParamTypeInt8:
        cf_.setParam<int8_t>(entry->id, p.as_int());
        break;
      case Crazyflie::ParamTypeUint16:
        cf_.setParam<uint16_t>(entry->id, p.as_int());
        break;
      case Crazyflie::ParamTypeInt16:
        cf_.setParam<int16_t>(entry->id, p.as_int());
        break;
      case Crazyflie::ParamTypeUint32:
        cf_.setParam<uint32_t>(entry->id, p.as_int());
        break;
      case Crazyflie::ParamTypeInt32:
        cf_.setParam<int32_t>(entry->id, p.as_int());
        break;
      case Crazyflie::ParamTypeFloat:
        if (p.get_type() == rclcpp::PARAMETER_INTEGER) {
          cf_.setParam<float>(entry->id, (float)p.as_int());
        } else {
          cf_.setParam<float>(entry->id, p.as_double());
        }

        break;
      }
    } else {
      RCLCPP_ERROR(logger_, "[%s] Could not find param %s/%s", name_.c_str(), group.c_str(), name.c_str());
    }
  }

private:

  void cmd_full_state_changed(const crazyflie_interfaces::msg::FullState::SharedPtr msg)
  {
    float x = msg->pose.position.x;
    float y = msg->pose.position.y;
    float z = msg->pose.position.z;
    float vx = msg->twist.linear.x;
    float vy = msg->twist.linear.y;
    float vz = msg->twist.linear.z;
    float ax = msg->acc.x;
    float ay = msg->acc.y;
    float az = msg->acc.z;

    float qx = msg->pose.orientation.x;
    float qy = msg->pose.orientation.y;
    float qz = msg->pose.orientation.z;
    float qw = msg->pose.orientation.w;
    float rollRate = msg->twist.angular.x;
    float pitchRate = msg->twist.angular.y;
    float yawRate = msg->twist.angular.z;
    cf_.sendFullStateSetpoint(
    x, y, z,
    vx, vy, vz,
    ax, ay, az,
    qx, qy, qz, qw,
    rollRate, pitchRate, yawRate);

  }

  void cmd_position_changed(const crazyflie_interfaces::msg::Position::SharedPtr msg) {
    float x = msg->x;
    float y = msg->y;
    float z = msg->z;
    float yaw = msg->yaw;
    cf_.sendPositionSetpoint(x, y, z, yaw);
  }

  void cmd_position_control_changed(const crazyflie_interfaces::msg::PositionControl::SharedPtr msg)
  {
    const uint8_t requested_mode =
      msg->position_mode == crazyflie_interfaces::msg::PositionControl::MODE_VELOCITY
        ? crazyflie_interfaces::msg::PositionControl::MODE_VELOCITY
        : crazyflie_interfaces::msg::PositionControl::MODE_POSITION;
    const uint8_t requested_trajectory =
      msg->trajectory_mode <= crazyflie_interfaces::msg::PositionControl::TRAJECTORY_2
        ? msg->trajectory_mode
        : crazyflie_interfaces::msg::PositionControl::TRAJECTORY_NONE;
    const uint8_t requested_reference =
      msg->command_reference == crazyflie_interfaces::msg::PositionControl::REFERENCE_END_EFFECTOR
        ? crazyflie_interfaces::msg::PositionControl::REFERENCE_END_EFFECTOR
        : crazyflie_interfaces::msg::PositionControl::REFERENCE_DRONE;

    const struct __attribute__((packed)) {
      uint8_t magic;
      uint8_t version;
      uint8_t mode;
      uint8_t trajectory;
      uint8_t reference;
      float force_desired;
    } payload = {
      kPositionControlTriggerMagic,
      kPositionControlTriggerVersion,
      requested_mode,
      requested_trajectory,
      requested_reference,
      msg->force_desired,
    };
    cf_.sendAppChannelPacket(reinterpret_cast<const uint8_t*>(&payload), sizeof(payload));
  }

  void hover_calibration_changed(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (!msg || msg->data.size() < 3) {
      RCLCPP_WARN(logger_, "[%s] hover_calibration ignored: expected [mass, comOffX, comOffY]", name_.c_str());
      return;
    }
    if (!has_hover_calibration_mass_ || hover_calibration_mass_ <= 0.0) {
      RCLCPP_ERROR(
        logger_,
        "[%s] hover_calibration ignored: no valid su_wrench.mass was loaded from configuration",
        name_.c_str());
      return;
    }

    const double mass = hover_calibration_mass_;
    const double com_off_x = msg->data[1];
    const double com_off_y = msg->data[2];

    const struct __attribute__((packed)) {
      uint8_t magic;
      uint8_t version;
      float mass;
      float com_off_x;
      float com_off_y;
    } payload = {
      kHoverCalibrationTriggerMagic,
      kHoverCalibrationTriggerVersion,
      static_cast<float>(mass),
      static_cast<float>(com_off_x),
      static_cast<float>(com_off_y),
    };

    cf_.sendAppChannelPacket(reinterpret_cast<const uint8_t*>(&payload), sizeof(payload));

    publish_debug_log_filename_tag(mass, com_off_x, com_off_y);
    persist_hover_calibration_to_yaml(com_off_x, com_off_y);
    RCLCPP_INFO(
      logger_,
      "[%s] hover_calibration forwarded to firmware: configured mass=%.4f kg, comOffXY=(%.5f, %.5f) m",
      name_.c_str(),
      mass,
      com_off_x,
      com_off_y);
  }

  void publish_debug_log_filename_tag(double mass, double com_off_x, double com_off_y)
  {
    if (!publisher_debug_log_filename_tag_) {
      return;
    }

    std::ostringstream tag_stream;
    tag_stream << std::fixed << std::setprecision(3)
               << "mass" << mass
               << "_com" << com_off_x
               << "_" << com_off_y;

    std_msgs::msg::String tag_msg;
    tag_msg.data = tag_stream.str();
    publisher_debug_log_filename_tag_->publish(tag_msg);

    RCLCPP_INFO(
      logger_,
      "[%s] published debug log filename tag: %s",
      name_.c_str(),
      tag_msg.data.c_str());
  }

  std::vector<std::filesystem::path> resolve_su_params_paths() const
  {
    std::vector<std::filesystem::path> paths;
    try {
      const auto share_dir = ament_index_cpp::get_package_share_directory("crazyflie");
      paths.emplace_back(std::filesystem::path(share_dir) / "config" / "su_params.yaml");
    } catch (const std::exception& e) {
      RCLCPP_WARN(logger_, "[%s] Could not resolve crazyflie share su_params.yaml: %s", name_.c_str(), e.what());
    }

    const char* home = std::getenv("HOME");
    if (home) {
      paths.emplace_back(
        std::filesystem::path(home) / "hitl_ws" / "src" / "crazyswarm2" / "crazyflie" / "config" / "su_params.yaml");
    }

    std::vector<std::filesystem::path> deduped;
    for (const auto& path : paths) {
      const auto duplicate =
        std::find(deduped.begin(), deduped.end(), path) != deduped.end();
      if (!duplicate) {
        deduped.push_back(path);
      }
    }
    return deduped;
  }

  bool update_su_params_file(
    const std::filesystem::path& path,
    double com_off_x,
    double com_off_y) const
  {
    if (!std::filesystem::exists(path)) {
      throw std::runtime_error("file does not exist");
    }

    std::ifstream in(path);
    if (!in.is_open()) {
      throw std::runtime_error("failed to open file for reading");
    }

    std::vector<std::string> lines;
    std::string line;
    while (std::getline(in, line)) {
      lines.push_back(line);
    }
    in.close();

    bool in_cf21 = false;
    bool in_firmware_params = false;
    bool in_su_wrench = false;
    bool found_com_x = false;
    bool found_com_y = false;

    for (auto& current_line : lines) {
      std::string stripped = current_line;
      stripped.erase(0, stripped.find_first_not_of(' '));
      const size_t indent = current_line.size() - stripped.size();

      if (indent == 2 && stripped == "cf21:") {
        in_cf21 = true;
        in_firmware_params = false;
        in_su_wrench = false;
        continue;
      }

      if (in_cf21 && indent <= 2 && !stripped.empty() && stripped.back() == ':' && stripped != "cf21:") {
        in_cf21 = false;
        in_firmware_params = false;
        in_su_wrench = false;
      }

      if (in_cf21 && indent == 4 && stripped == "firmware_params:") {
        in_firmware_params = true;
        in_su_wrench = false;
        continue;
      }

      if (in_firmware_params && indent <= 4 && !stripped.empty() && stripped.back() == ':' && stripped != "firmware_params:") {
        in_firmware_params = false;
        in_su_wrench = false;
      }

      if (in_firmware_params && indent == 6 && stripped == "su_wrench:") {
        in_su_wrench = true;
        continue;
      }

      if (in_su_wrench && indent <= 6 && !stripped.empty() && stripped.back() == ':' && stripped != "su_wrench:") {
        in_su_wrench = false;
      }

      if (!in_su_wrench) {
        continue;
      }

      if (indent == 8 && stripped.rfind("comOffX:", 0) == 0) {
        current_line = replace_yaml_scalar_line(current_line, com_off_x);
        found_com_x = true;
      } else if (indent == 8 && stripped.rfind("comOffY:", 0) == 0) {
        current_line = replace_yaml_scalar_line(current_line, com_off_y);
        found_com_y = true;
      }
    }

    if (!(found_com_x && found_com_y)) {
      return false;
    }

    std::ofstream out(path, std::ios::out | std::ios::trunc);
    if (!out.is_open()) {
      throw std::runtime_error("failed to open file for writing");
    }

    for (const auto& updated_line : lines) {
      out << updated_line << "\n";
    }
    return true;
  }

  void persist_hover_calibration_to_yaml(double com_off_x, double com_off_y)
  {
    std::vector<std::string> updated_paths;
    std::vector<std::string> failed_paths;

    for (const auto& path : resolve_su_params_paths()) {
      try {
        if (update_su_params_file(path, com_off_x, com_off_y)) {
          updated_paths.push_back(path.string());
        } else {
          failed_paths.push_back(path.string() + " (keys not found)");
        }
      } catch (const std::exception& e) {
        failed_paths.push_back(path.string() + " (" + e.what() + ")");
      }
    }

    if (!updated_paths.empty()) {
      std::ostringstream oss;
      for (size_t i = 0; i < updated_paths.size(); ++i) {
        if (i > 0) {
          oss << " | ";
        }
        oss << updated_paths[i];
      }
      RCLCPP_INFO(
        logger_,
        "[%s] persisted hover com offsets to su_params.yaml: %s",
        name_.c_str(),
        oss.str().c_str());
    }

    if (!failed_paths.empty()) {
      std::ostringstream oss;
      for (size_t i = 0; i < failed_paths.size(); ++i) {
        if (i > 0) {
          oss << " | ";
        }
        oss << failed_paths[i];
      }
      RCLCPP_WARN(
        logger_,
        "[%s] failed to persist some hover com offset updates: %s",
        name_.c_str(),
        oss.str().c_str());
    }
  }

  void cmd_hover_changed(const crazyflie_interfaces::msg::Hover::SharedPtr msg)
  {
    float vx = msg->vx;
    float vy = msg->vy;
    float yawrate = msg->yaw_rate;
    float z_distance = msg->z_distance;
    cf_.sendHoverSetpoint(vx, vy, yawrate, z_distance);
  }

  void cmd_velocity_world_changed(const crazyflie_interfaces::msg::VelocityWorld::SharedPtr msg)
  {
    float vx = msg->vel.x;
    float vy = msg->vel.y;
    float vz = msg->vel.z;
    float yawrate = msg->yaw_rate;
    cf_.sendVelocityWorldSetpoint(vx, vy, vz, yawrate);
  }

  void cmd_vel_legacy_changed(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    float roll = msg->linear.y;
    float pitch = - (msg->linear.x);
    float yawrate = msg->angular.z;
    uint16_t thrust = std::min<uint16_t>(std::max<float>(msg->linear.z, 0.0), 60000);
    // RCLCPP_INFO(logger_, "roll: %f, pitch: %f, yaw: %f, thrust: %u", roll, pitch, yawrate, (unsigned int)thrust);
    cf_.sendSetpoint(roll, pitch, yawrate, thrust);
  }

  void on_console(const char *msg)
  {
    message_buffer_ += msg;
    size_t pos = message_buffer_.find('\n');
    if (pos != std::string::npos)
    {
      message_buffer_[pos] = 0;
      RCLCPP_INFO(logger_, "[%s] %s", name_.c_str(), message_buffer_.c_str());
      message_buffer_.erase(0, pos + 1);
    }
  }

  void emergency(const std::shared_ptr<Empty::Request> request,
            std::shared_ptr<Empty::Response> response)
  {
    RCLCPP_INFO(logger_, "[%s] emergency()", name_.c_str());
    cf_.emergencyStop();
  }

  void start_trajectory(const std::shared_ptr<StartTrajectory::Request> request,
                        std::shared_ptr<StartTrajectory::Response> response)
  {
    RCLCPP_INFO(logger_, "[%s] start_trajectory(id=%d, timescale=%f, reversed=%d, relative=%d, group_mask=%d)",
      name_.c_str(),
      request->trajectory_id,
      request->timescale,
      request->reversed,
      request->relative,
      request->group_mask);
    cf_.startTrajectory(request->trajectory_id,
      request->timescale,
      request->reversed,
      request->relative,
      request->group_mask);
  }

  void takeoff(const std::shared_ptr<Takeoff::Request> request,
               std::shared_ptr<Takeoff::Response> response)
  {
    RCLCPP_INFO(logger_, "[%s] takeoff(height=%f m, duration=%f s, group_mask=%d)", 
                name_.c_str(),
                request->height,
                rclcpp::Duration(request->duration).seconds(),
                request->group_mask);
    cf_.takeoff(request->height, rclcpp::Duration(request->duration).seconds(), request->group_mask);
  }

  void land(const std::shared_ptr<Land::Request> request,
            std::shared_ptr<Land::Response> response)
  {
    RCLCPP_INFO(logger_, "[%s] land(height=%f m, duration=%f s, group_mask=%d)",
                name_.c_str(),
                request->height,
                rclcpp::Duration(request->duration).seconds(),
                request->group_mask);
    cf_.land(request->height, rclcpp::Duration(request->duration).seconds(), request->group_mask);
  }

  void go_to(const std::shared_ptr<GoTo::Request> request,
             std::shared_ptr<GoTo::Response> response)
  {
    RCLCPP_INFO(logger_, "[%s] go_to(position=%f,%f,%f m, yaw=%f rad, duration=%f s, relative=%d, group_mask=%d)",
                name_.c_str(),
                request->goal.x, request->goal.y, request->goal.z, request->yaw,
                rclcpp::Duration(request->duration).seconds(),
                request->relative,
                request->group_mask);
    cf_.goTo(request->goal.x, request->goal.y, request->goal.z, request->yaw, 
              rclcpp::Duration(request->duration).seconds(),
              request->relative, request->group_mask);
  }

  void upload_trajectory(const std::shared_ptr<UploadTrajectory::Request> request,
                        std::shared_ptr<UploadTrajectory::Response> response)
  {
    RCLCPP_INFO(logger_, "[%s] upload_trajectory(id=%d, offset=%d)",
                name_.c_str(),
                request->trajectory_id,
                request->piece_offset);

    std::vector<Crazyflie::poly4d> pieces(request->pieces.size());
    for (size_t i = 0; i < pieces.size(); ++i)
    {
      if (   request->pieces[i].poly_x.size() != 8 
          || request->pieces[i].poly_y.size() != 8
          || request->pieces[i].poly_z.size() != 8
          || request->pieces[i].poly_yaw.size() != 8)
      {
        RCLCPP_FATAL(logger_, "[%s] Wrong number of pieces!", name_.c_str());
        return;
      }
      pieces[i].duration = rclcpp::Duration(request->pieces[i].duration).seconds();
      for (size_t j = 0; j < 8; ++j)
      {
        pieces[i].p[0][j] = request->pieces[i].poly_x[j];
        pieces[i].p[1][j] = request->pieces[i].poly_y[j];
        pieces[i].p[2][j] = request->pieces[i].poly_z[j];
        pieces[i].p[3][j] = request->pieces[i].poly_yaw[j];
      }
    }
    cf_.uploadTrajectory(request->trajectory_id, request->piece_offset, pieces);
  }

  void notify_setpoints_stop(const std::shared_ptr<NotifySetpointsStop::Request> request,
                         std::shared_ptr<NotifySetpointsStop::Response> response)
  {
    RCLCPP_INFO(logger_, "[%s] notify_setpoints_stop(remain_valid_millisecs%d, group_mask=%d)",
                name_.c_str(),
                request->remain_valid_millisecs,
                request->group_mask);

    cf_.notifySetpointsStop(request->remain_valid_millisecs);
  }

  void arm(const std::shared_ptr<Arm::Request> request,
                         std::shared_ptr<Arm::Response> response)
  {
    RCLCPP_INFO(logger_, "[%s] arm(%d)",
                name_.c_str(),
                request->arm);

    cf_.sendArmingRequest(request->arm);
  }

  void on_logging_pose(uint32_t time_in_ms, const logPose* data) {
    if (publisher_pose_) {
      geometry_msgs::msg::PoseStamped msg;
      msg.header.stamp = node_->get_clock()->now();
      msg.header.frame_id = reference_frame_;

      msg.pose.position.x = data->x;
      msg.pose.position.y = data->y;
      msg.pose.position.z = data->z;

      float q[4];
      quatdecompress(data->quatCompressed, q);
      msg.pose.orientation.x = q[0];
      msg.pose.orientation.y = q[1];
      msg.pose.orientation.z = q[2];
      msg.pose.orientation.w = q[3];

      publisher_pose_->publish(msg);

      // send a transform for this pose
      geometry_msgs::msg::TransformStamped msg2;
      msg2.header = msg.header;
      msg2.child_frame_id = name_;
      msg2.transform.translation.x = data->x;
      msg2.transform.translation.y = data->y;
      msg2.transform.translation.z = data->z;
      msg2.transform.rotation.x = q[0];
      msg2.transform.rotation.y = q[1];
      msg2.transform.rotation.z = q[2];
      msg2.transform.rotation.w = q[3];
      tf_broadcaster_.sendTransform(msg2);
    }
  }

  void on_logging_scan(uint32_t time_in_ms, const logScan* data) {
    if (publisher_scan_) {
      
      const float max_range = 3.49;
      float front_range = data->front / 1000.0f;
      if (front_range > max_range) front_range = std::numeric_limits<float>::infinity();
      float left_range = data->left / 1000.0f;
      if (left_range > max_range) left_range = std::numeric_limits<float>::infinity();
      float back_range = data->back / 1000.0f;
      if (back_range > max_range) back_range = std::numeric_limits<float>::infinity();
      float right_range = data->right / 1000.0f;
      if (right_range > max_range) right_range = std::numeric_limits<float>::infinity();

      sensor_msgs::msg::LaserScan msg;
      msg.header.stamp = node_->get_clock()->now();
      msg.header.frame_id = name_;
      msg.range_min = 0.01;
      msg.range_max = max_range;
      msg.ranges.push_back(back_range);
      msg.ranges.push_back(right_range);
      msg.ranges.push_back(front_range);
      msg.ranges.push_back(left_range);
      msg.angle_min = -0.5 * 2 * M_PI;
      msg.angle_max = 0.25 * 2 * M_PI;
      msg.angle_increment = 1.0 * M_PI / 2;

      publisher_scan_->publish(msg);
    }
  }

  void on_logging_status(uint32_t time_in_ms, const logStatus* data) {
    if (publisher_status_) {
      
      crazyflie_interfaces::msg::Status msg;
      msg.header.stamp = node_->get_clock()->now();
      msg.header.frame_id = name_;
      msg.supervisor_info = data->supervisorInfo;
      msg.battery_voltage = data->vbatMV / 1000.0f;
      msg.pm_state = data->pmState;
      msg.rssi = data->rssi;
      if (status_has_radio_stats_) {
        int32_t deltaRxBc = data->numRxBc - previous_numRxBc;
        int32_t deltaRxUc = data->numRxUc - previous_numRxUc;
        // handle overflow
        if (deltaRxBc < 0) {
          deltaRxBc += std::numeric_limits<uint16_t>::max();
        }
        if (deltaRxUc < 0) {
          deltaRxUc += std::numeric_limits<uint16_t>::max();
        }
        msg.num_rx_broadcast = deltaRxBc;
        msg.num_rx_unicast = deltaRxUc;
        previous_numRxBc = data->numRxBc;
        previous_numRxUc = data->numRxUc;
      } else {
        msg.num_rx_broadcast = 0;
        msg.num_rx_unicast = 0;
      }

      // connection sent stats (unicast)
      const auto statsUc = cf_.connectionStats();
      size_t deltaTxUc = statsUc.sent_count - previous_stats_unicast_.sent_count;
      msg.num_tx_unicast = deltaTxUc;
      previous_stats_unicast_ = statsUc;

      // connection sent stats (broadcast)
      const auto statsBc = cfbc_->connectionStats();
      size_t deltaTxBc = statsBc.sent_count - previous_stats_broadcast_.sent_count;
      msg.num_tx_broadcast = deltaTxBc;
      previous_stats_broadcast_ = statsBc;

      msg.latency_unicast = last_latency_in_ms_;

      publisher_status_->publish(msg);

      // warnings
      if (msg.num_rx_unicast > msg.num_tx_unicast * 1.05 /*allow some slack*/) {
        RCLCPP_WARN(logger_, "[%s] Unexpected number of unicast packets. Sent: %d. Received: %d", name_.c_str(), msg.num_tx_unicast, msg.num_rx_unicast);
      }
      if (msg.num_tx_unicast > 0) {
        float unicast_receive_rate = msg.num_rx_unicast / (float)msg.num_tx_unicast;
        if (unicast_receive_rate < min_unicast_receive_rate_) {
          RCLCPP_WARN(logger_, "[%s] Low unicast receive rate (%.2f < %.2f). Sent: %d. Received: %d", name_.c_str(), unicast_receive_rate, min_unicast_receive_rate_, msg.num_tx_unicast, msg.num_rx_unicast);
        }
      }

      if (msg.num_rx_broadcast > msg.num_tx_broadcast * 1.05 /*allow some slack*/) {
        RCLCPP_WARN(logger_, "[%s] Unexpected number of broadcast packets. Sent: %d. Received: %d", name_.c_str(), msg.num_tx_broadcast, msg.num_rx_broadcast);
      }
      if (msg.num_tx_broadcast > 0) {
        float broadcast_receive_rate = msg.num_rx_broadcast / (float)msg.num_tx_broadcast;
        if (broadcast_receive_rate < min_broadcast_receive_rate_) {
          RCLCPP_WARN(logger_, "[%s] Low broadcast receive rate (%.2f < %.2f). Sent: %d. Received: %d", name_.c_str(), broadcast_receive_rate, min_broadcast_receive_rate_, msg.num_tx_broadcast, msg.num_rx_broadcast);
        }
      }
    }
  }

  void on_logging_custom(uint32_t time_in_ms, const std::vector<float>* values, void* userData) {

    auto pub = reinterpret_cast<rclcpp::Publisher<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr*>(userData);

    crazyflie_interfaces::msg::LogDataGeneric msg;
    msg.header.stamp = node_->get_clock()->now();
    msg.header.frame_id = reference_frame_;
    msg.timestamp = time_in_ms;
    msg.values = *values;

    (*pub)->publish(msg);
  }

  void on_link_statistics_timer()
  {
    cf_.triggerLatencyMeasurement();

    auto now = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed = now - last_on_latency_;
    if (elapsed.count() > 1.0 / warning_freq_) {
      RCLCPP_WARN(logger_, "[%s] last latency update: %f s", name_.c_str(), elapsed.count());
    }

    auto stats = cf_.connectionStatsDelta();

    if (stats.ack_count > 0) {
      float ack_rate = stats.sent_count / stats.ack_count;
      if (ack_rate < min_ack_rate_) {
        RCLCPP_WARN(logger_, "[%s] Ack rate: %.1f %%", name_.c_str(), ack_rate * 100);
      }
    }

    if (publish_stats_) {
      crazyflie_interfaces::msg::ConnectionStatisticsArray msg;
      msg.header.stamp = node_->get_clock()->now();
      msg.header.frame_id = reference_frame_;
      msg.stats.resize(1);

      msg.stats[0].uri = cf_.uri();
      msg.stats[0].sent_count = stats.sent_count;
      msg.stats[0].sent_ping_count = stats.sent_ping_count;
      msg.stats[0].receive_count = stats.receive_count;
      msg.stats[0].enqueued_count = stats.enqueued_count;
      msg.stats[0].ack_count = stats.ack_count;

      publisher_connection_stats_->publish(msg);
    }
  }

  void on_latency(uint64_t latency_in_us)
  {
    if (latency_in_us / 1000.0 > max_latency_) {
      RCLCPP_WARN(logger_, "[%s] High latency: %.1f ms", name_.c_str(), latency_in_us / 1000.0);
    }
    last_on_latency_ = std::chrono::steady_clock::now();
    last_latency_in_ms_ = (uint16_t)(latency_in_us / 1000.0);
  }

private:
  rclcpp::Logger logger_;
  CrazyflieLogger cf_logger_;

  Crazyflie cf_;
  std::string message_buffer_;
  std::string name_;
  double hover_calibration_mass_{0.0};
  bool has_hover_calibration_mass_{false};

  rclcpp::Node* node_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  rclcpp::Service<Empty>::SharedPtr service_emergency_;
  rclcpp::Service<StartTrajectory>::SharedPtr service_start_trajectory_;
  rclcpp::Service<Takeoff>::SharedPtr service_takeoff_;
  rclcpp::Service<Land>::SharedPtr service_land_;
  rclcpp::Service<GoTo>::SharedPtr service_go_to_;
  rclcpp::Service<UploadTrajectory>::SharedPtr service_upload_trajectory_;
  rclcpp::Service<NotifySetpointsStop>::SharedPtr service_notify_setpoints_stop_;
  rclcpp::Service<Arm>::SharedPtr service_arm_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_cmd_vel_legacy_;
  rclcpp::Subscription<crazyflie_interfaces::msg::FullState>::SharedPtr subscription_cmd_full_state_;
  rclcpp::Subscription<crazyflie_interfaces::msg::Hover>::SharedPtr subscription_cmd_hover_;
  rclcpp::Subscription<crazyflie_interfaces::msg::Position>::SharedPtr subscription_cmd_position_;
  rclcpp::Subscription<crazyflie_interfaces::msg::PositionControl>::SharedPtr subscription_cmd_position_control_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_hover_calibration_;
  rclcpp::Subscription<crazyflie_interfaces::msg::VelocityWorld>::SharedPtr subscription_cmd_velocity_world_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_robot_description_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_debug_log_filename_tag_;

  // logging
  std::string reference_frame_;

  std::unique_ptr<LogBlock<logPose>> log_block_pose_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_pose_;

  std::unique_ptr<LogBlock<logScan>> log_block_scan_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_scan_;

  std::unique_ptr<LogBlock<logStatus>> log_block_status_;
  bool status_has_radio_stats_;
  rclcpp::Publisher<crazyflie_interfaces::msg::Status>::SharedPtr publisher_status_;
  uint16_t previous_numRxBc;
  uint16_t previous_numRxUc;
  bitcraze::crazyflieLinkCpp::Connection::Statistics previous_stats_unicast_;
  bitcraze::crazyflieLinkCpp::Connection::Statistics previous_stats_broadcast_;
  const CrazyflieBroadcaster* cfbc_;

  std::list<std::unique_ptr<LogBlockGeneric>> log_blocks_generic_;
  std::list<rclcpp::Publisher<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr> publishers_generic_;

  // multithreading
  rclcpp::CallbackGroup::SharedPtr callback_group_cf_;
  rclcpp::TimerBase::SharedPtr spin_timer_;

  // link statistics
  rclcpp::TimerBase::SharedPtr link_statistics_timer_;
  std::chrono::time_point<std::chrono::steady_clock> last_on_latency_;
  uint16_t last_latency_in_ms_;
  float warning_freq_;
  float max_latency_;
  float min_ack_rate_;
  float min_unicast_receive_rate_;
  float min_broadcast_receive_rate_;
  bool publish_stats_;
  rclcpp::Publisher<crazyflie_interfaces::msg::ConnectionStatisticsArray>::SharedPtr publisher_connection_stats_;
};

class CrazyflieServer : public rclcpp::Node
{
public:
  CrazyflieServer()
      : Node("crazyflie_server")
      , logger_(get_logger())
  {
    // Create callback groups (each group can run in a separate thread)
    callback_group_mocap_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    auto sub_opt_mocap = rclcpp::SubscriptionOptions();
    sub_opt_mocap.callback_group = callback_group_mocap_;

    callback_group_all_cmd_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    auto sub_opt_all_cmd = rclcpp::SubscriptionOptions();
    sub_opt_all_cmd.callback_group = callback_group_all_cmd_;

    callback_group_all_srv_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

    callback_group_cf_cmd_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

    callback_group_cf_srv_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

    // declare global params
    this->declare_parameter("all.broadcasts.num_repeats", 15);
    this->declare_parameter("all.broadcasts.delay_between_repeats_ms", 1);
    this->declare_parameter("firmware_params.query_all_values_on_connect", false);

    broadcasts_num_repeats_ = this->get_parameter("all.broadcasts.num_repeats").get_parameter_value().get<int>();
    broadcasts_delay_between_repeats_ms_ = this->get_parameter("all.broadcasts.delay_between_repeats_ms").get_parameter_value().get<int>();
    mocap_enabled_ = false;

    this->declare_parameter("robot_description", "");

    // Warnings
    this->declare_parameter("warnings.frequency", 1.0);
    float freq = this->get_parameter("warnings.frequency").get_parameter_value().get<float>();
    if (freq >= 0.0) {
      watchdog_timer_ = this->create_wall_timer(std::chrono::milliseconds((int)(1000.0/freq)), std::bind(&CrazyflieServer::on_watchdog_timer, this), callback_group_all_srv_);
    }
    this->declare_parameter("warnings.motion_capture.warning_if_rate_outside", std::vector<double>({80.0, 120.0}));
    auto rate_range = this->get_parameter("warnings.motion_capture.warning_if_rate_outside").get_parameter_value().get<std::vector<double>>();
    mocap_min_rate_ = rate_range[0];
    mocap_max_rate_ = rate_range[1];

    this->declare_parameter("warnings.communication.max_unicast_latency", 10.0);
    this->declare_parameter("warnings.communication.min_unicast_ack_rate", 0.9);
    this->declare_parameter("warnings.communication.min_unicast_receive_rate", 0.9);
    this->declare_parameter("warnings.communication.min_broadcast_receive_rate", 0.9);
    this->declare_parameter("warnings.communication.publish_stats", false);

    publish_stats_ = this->get_parameter("warnings.communication.publish_stats").get_parameter_value().get<bool>();
    if (publish_stats_) {
      publisher_connection_stats_ = this->create_publisher<crazyflie_interfaces::msg::ConnectionStatisticsArray>("all/connection_statistics", 10);
    }

    // load crazyflies from params
    auto node_parameters_iface = this->get_node_parameters_interface();
    const std::map<std::string, rclcpp::ParameterValue> &parameter_overrides =
        node_parameters_iface->get_parameter_overrides();

    auto cf_names = extract_names(parameter_overrides, "robots");
    for (const auto &name : cf_names) {
      bool enabled = parameter_overrides.at("robots." + name + ".enabled").get<bool>();
      if (enabled) {
        // Lookup type
        std::string cf_type = parameter_overrides.at("robots." + name + ".type").get<std::string>();
        // Find the connection setting for the given type
        const auto con = parameter_overrides.find("robot_types." + cf_type + ".connection");
        std::string constr = "crazyflie";
        if (con != parameter_overrides.end()) {
          constr = con->second.get<std::string>();
        }
        // Find the mocap setting
        const auto mocap_en = parameter_overrides.find("robot_types." + cf_type + ".motion_capture.enabled");
        if (mocap_en != parameter_overrides.end()) {
          if (mocap_en->second.get<bool>()) {
            mocap_enabled_ = true;
          }
        }

        // if it is a Crazyflie, try to connect
        if (constr == "crazyflie") {
          std::string uri = parameter_overrides.at("robots." + name + ".uri").get<std::string>();
          auto broadcastUri = Crazyflie::broadcastUriFromUnicastUri(uri);
          if (broadcaster_.count(broadcastUri) == 0) {
            broadcaster_.emplace(broadcastUri, std::make_unique<CrazyflieBroadcaster>(broadcastUri));
          }

          crazyflies_.emplace(name, std::make_unique<CrazyflieROS>(
            uri,
            cf_type,
            name,
            this,
            callback_group_cf_cmd_,
            callback_group_cf_srv_,
            broadcaster_.at(broadcastUri).get()));

          update_name_to_id_map(name, crazyflies_[name]->id());
        }
        else if (constr == "none") {
          // we still might want to track this object, so update our map
          uint8_t id = parameter_overrides.at("robots." + name + ".id").get<uint8_t>();
          update_name_to_id_map(name, id);
        } else {
          RCLCPP_INFO(logger_, "[all] Unknown connection type %s", constr.c_str());
        }
      }
    }

    this->declare_parameter("poses_qos_deadline", 100.0f);
    double poses_qos_deadline = this->get_parameter("poses_qos_deadline").get_parameter_value().get<double>();

    rclcpp::SensorDataQoS sensor_data_qos;
    sensor_data_qos.keep_last(1);
    sensor_data_qos.deadline(rclcpp::Duration(0/*s*/, 1e9/poses_qos_deadline /*ns*/));
    sub_poses_ = this->create_subscription<NamedPoseArray>(
        "poses", sensor_data_qos, std::bind(&CrazyflieServer::posesChanged, this, _1), sub_opt_mocap);

    // support for all.params

    // Create a parameter subscriber that can be used to monitor parameter changes
    param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
    cb_handle_ = param_subscriber_->add_parameter_event_callback(std::bind(&CrazyflieServer::on_parameter_event, this, _1));

    // topics for "all"
    subscription_cmd_full_state_ = this->create_subscription<crazyflie_interfaces::msg::FullState>("all/cmd_full_state", rclcpp::SystemDefaultsQoS(), std::bind(&CrazyflieServer::cmd_full_state_changed, this, _1), sub_opt_all_cmd);
    subscription_cmd_hover_ = this->create_subscription<crazyflie_interfaces::msg::Hover>("all/cmd_hover", rclcpp::SystemDefaultsQoS(), std::bind(&CrazyflieServer::cmd_hover_changed, this, _1), sub_opt_all_cmd);
    subscription_cmd_velocity_world_ = this->create_subscription<crazyflie_interfaces::msg::VelocityWorld>("all/cmd_velocity_world", rclcpp::SystemDefaultsQoS(), std::bind(&CrazyflieServer::cmd_velocity_world_changed, this, _1), sub_opt_all_cmd);

    // services for "all"
    service_start_trajectory_ = this->create_service<StartTrajectory>("all/start_trajectory", std::bind(&CrazyflieServer::start_trajectory, this, _1, _2), get_service_qos(), callback_group_all_srv_);
    service_takeoff_ = this->create_service<Takeoff>("all/takeoff", std::bind(&CrazyflieServer::takeoff, this, _1, _2), get_service_qos(), callback_group_all_srv_);
    service_land_ = this->create_service<Land>("all/land", std::bind(&CrazyflieServer::land, this, _1, _2), get_service_qos(), callback_group_all_srv_);
    service_go_to_ = this->create_service<GoTo>("all/go_to", std::bind(&CrazyflieServer::go_to, this, _1, _2), get_service_qos(), callback_group_all_srv_);
    service_notify_setpoints_stop_ = this->create_service<NotifySetpointsStop>("all/notify_setpoints_stop", std::bind(&CrazyflieServer::notify_setpoints_stop, this, _1, _2), get_service_qos(), callback_group_all_srv_);
    service_arm_ = this->create_service<Arm>("all/arm", std::bind(&CrazyflieServer::arm, this, _1, _2), get_service_qos(), callback_group_all_srv_);

    // This is the last service to announce and can be used to check if the server is fully available
    service_emergency_ = this->create_service<Empty>("all/emergency", std::bind(&CrazyflieServer::emergency, this, _1, _2), get_service_qos(), callback_group_all_srv_);
  }


private:
  void emergency(const std::shared_ptr<Empty::Request> request,
            std::shared_ptr<Empty::Response> response)
  {
    RCLCPP_INFO(logger_, "[all] emergency()");
    for (int i = 0; i < broadcasts_num_repeats_; ++i)
    {
      for (auto &bc : broadcaster_) {
        auto &cfbc = bc.second;
        cfbc->emergencyStop();
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(broadcasts_delay_between_repeats_ms_));
    }
  }

  void start_trajectory(const std::shared_ptr<StartTrajectory::Request> request,
            std::shared_ptr<StartTrajectory::Response> response)
  {
    RCLCPP_INFO(logger_, "[all] start_trajectory(id=%d, timescale=%f, reversed=%d, group_mask=%d)",
                request->trajectory_id,
                request->timescale,
                request->reversed,
                request->group_mask);
    for (int i = 0; i < broadcasts_num_repeats_; ++i) {
      for (auto &bc : broadcaster_) {
        auto &cfbc = bc.second;
        cfbc->startTrajectory(request->trajectory_id,
                            request->timescale,
                            request->reversed,
                            request->group_mask);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(broadcasts_delay_between_repeats_ms_));
    }
  }

  void takeoff(const std::shared_ptr<Takeoff::Request> request,
                        std::shared_ptr<Takeoff::Response> response)
  {
    RCLCPP_INFO(logger_, "[all] takeoff(height=%f m, duration=%f s, group_mask=%d)",
                request->height,
                rclcpp::Duration(request->duration).seconds(),
                request->group_mask);
    for (int i = 0; i < broadcasts_num_repeats_; ++i) {
      for (auto& bc : broadcaster_) {
        auto& cfbc = bc.second;
        cfbc->takeoff(request->height, rclcpp::Duration(request->duration).seconds(), request->group_mask);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(broadcasts_delay_between_repeats_ms_));
    }
  }

  void land(const std::shared_ptr<Land::Request> request,
           std::shared_ptr<Land::Response> response)
  {
    RCLCPP_INFO(logger_, "[all] land(height=%f m, duration=%f s, group_mask=%d)",
                request->height,
                rclcpp::Duration(request->duration).seconds(),
                request->group_mask);
    for (int i = 0; i < broadcasts_num_repeats_; ++i) {
      for (auto& bc : broadcaster_) {
        auto& cfbc = bc.second;
        cfbc->land(request->height, rclcpp::Duration(request->duration).seconds(), request->group_mask);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(broadcasts_delay_between_repeats_ms_));
    }
  }

  void go_to(const std::shared_ptr<GoTo::Request> request,
            std::shared_ptr<GoTo::Response> response)
  {
    RCLCPP_INFO(logger_, "[all] go_to(position=%f,%f,%f m, yaw=%f rad, duration=%f s, group_mask=%d)",
                request->goal.x, request->goal.y, request->goal.z, request->yaw,
                rclcpp::Duration(request->duration).seconds(),
                request->group_mask);
    for (int i = 0; i < broadcasts_num_repeats_; ++i) {
      for (auto &bc : broadcaster_) {
        auto &cfbc = bc.second;
        cfbc->goTo(request->goal.x, request->goal.y, request->goal.z, request->yaw,
                rclcpp::Duration(request->duration).seconds(),
                request->group_mask);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(broadcasts_delay_between_repeats_ms_));
    }
  }

  void notify_setpoints_stop(const std::shared_ptr<NotifySetpointsStop::Request> request,
                         std::shared_ptr<NotifySetpointsStop::Response> response)
  {
    RCLCPP_INFO(logger_, "[all] notify_setpoints_stop(remain_valid_millisecs%d, group_mask=%d)",
                request->remain_valid_millisecs,
                request->group_mask);

    for (int i = 0; i < broadcasts_num_repeats_; ++i) {
      for (auto &bc : broadcaster_) {
        auto &cfbc = bc.second;
        cfbc->notifySetpointsStop(request->remain_valid_millisecs);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(broadcasts_delay_between_repeats_ms_));
    }
  }

  void arm(const std::shared_ptr<Arm::Request> request,
                         std::shared_ptr<Arm::Response> response)
  {
    RCLCPP_INFO(logger_, "[all] arm(%d)",
                request->arm);

    for (int i = 0; i < broadcasts_num_repeats_; ++i) {
      for (auto &bc : broadcaster_) {
        auto &cfbc = bc.second;
        cfbc->sendArmingRequest(request->arm);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(broadcasts_delay_between_repeats_ms_));
    }
  }

  void cmd_full_state_changed(const crazyflie_interfaces::msg::FullState::SharedPtr msg)
  { 
    float x = msg->pose.position.x;
    float y = msg->pose.position.y;
    float z = msg->pose.position.z;
    float vx = msg->twist.linear.x;
    float vy = msg->twist.linear.y;
    float vz = msg->twist.linear.z;
    float ax = msg->acc.x;
    float ay = msg->acc.y;
    float az = msg->acc.z;

    float qx = msg->pose.orientation.x;
    float qy = msg->pose.orientation.y;
    float qz = msg->pose.orientation.z;
    float qw = msg->pose.orientation.w;
    float rollRate = msg->twist.angular.x;
    float pitchRate = msg->twist.angular.y;
    float yawRate = msg->twist.angular.z;

    for (auto &bc : broadcaster_) {
        auto &cfbc = bc.second;
        cfbc->sendFullStateSetpoint(
          x, y, z,
          vx, vy, vz,
          ax, ay, az,
          qx, qy, qz, qw,
          rollRate, pitchRate, yawRate);
    }

  }

  void cmd_hover_changed(const crazyflie_interfaces::msg::Hover::SharedPtr msg)
  {
    float vx = msg->vx;
    float vy = msg->vy;
    float yawrate = msg->yaw_rate;
    float z_distance = msg->z_distance;

    for (auto &bc : broadcaster_) {
      auto &cfbc = bc.second;
      cfbc->sendHoverSetpoint(vx, vy, yawrate, z_distance);
    }
  }

  void cmd_velocity_world_changed(const crazyflie_interfaces::msg::VelocityWorld::SharedPtr msg)
  {
    float vx = msg->vel.x;
    float vy = msg->vel.y;
    float vz = msg->vel.z;
    float yawrate = msg->yaw_rate;

    for (auto &bc : broadcaster_) {
      auto &cfbc = bc.second;
      cfbc->sendVelocityWorldSetpoint(vx, vy, vz, yawrate);
    }
  }

  void posesChanged(const NamedPoseArray::SharedPtr msg)
  {
    mocap_data_received_timepoints_.emplace_back(std::chrono::steady_clock::now());

    // Here, we send all the poses to all CFs
    // In Crazyswarm1, we only sent the poses of the same group (i.e. channel)


    // split the message into parts that require position update and pose update
    std::vector<CrazyflieBroadcaster::externalPosition> data_position;
    std::vector<CrazyflieBroadcaster::externalPose> data_pose;

    for (const auto& pose : msg->poses) {
      const auto iter = name_to_id_.find(pose.name);
      if (iter != name_to_id_.end()) {
        uint8_t id = iter->second;
        if (isnan(pose.pose.orientation.w)) {
          data_position.push_back({id, 
            (float)pose.pose.position.x, (float)pose.pose.position.y, (float)pose.pose.position.z});
        } else {
          data_pose.push_back({id, 
            (float)pose.pose.position.x, (float)pose.pose.position.y, (float)pose.pose.position.z,
            (float)pose.pose.orientation.x, (float)pose.pose.orientation.y, (float)pose.pose.orientation.z, (float)pose.pose.orientation.w});
        }
      }
    }

    // send position only updates to the swarm
    if (data_position.size() > 0) {
      for (auto &bc : broadcaster_) {
        auto &cfbc = bc.second;
        cfbc->sendExternalPositions(data_position);
      }
    }

    // send pose only updates to the swarm
    if (data_pose.size() > 0) {
      for (auto &bc : broadcaster_) {
        auto &cfbc = bc.second;
        cfbc->sendExternalPoses(data_pose);
      }
    }
  }

  void on_parameter_event(const rcl_interfaces::msg::ParameterEvent &event)
  {
    if (event.node == "/crazyflie_server") {
      auto params = param_subscriber_->get_parameters_from_event(event);
      for (auto &p : params) {
        size_t params_pos = p.get_name().find(".params.");
        if (params_pos == std::string::npos) {
          continue;
        }
        std::string cfname(p.get_name().begin(), p.get_name().begin() + params_pos);
        size_t prefixsize = params_pos + 8;
        if (cfname == "all") {
          size_t pos = p.get_name().find(".", prefixsize);
          std::string group(p.get_name().begin() + prefixsize, p.get_name().begin() + pos);
          std::string name(p.get_name().begin() + pos + 1, p.get_name().end());

          RCLCPP_INFO(
              logger_,
              "[all] Update parameter \"%s.%s\" to %s",
              group.c_str(),
              name.c_str(),
              p.value_to_string().c_str());

          Crazyflie::ParamType paramType;
          for (auto& cf : crazyflies_) {
            const auto entry = cf.second->paramTocEntry(group, name);
            if (entry) {
              switch (entry->type)
              {
              case Crazyflie::ParamTypeUint8:
                broadcast_set_param<uint8_t>(group, name, p.as_int());
                break;
              case Crazyflie::ParamTypeInt8:
                broadcast_set_param<int8_t>(group, name, p.as_int());
                break;
              case Crazyflie::ParamTypeUint16:
                broadcast_set_param<uint16_t>(group, name, p.as_int());
                break;
              case Crazyflie::ParamTypeInt16:
                broadcast_set_param<int16_t>(group, name, p.as_int());
                break;
              case Crazyflie::ParamTypeUint32:
                broadcast_set_param<uint32_t>(group, name, p.as_int());
                break;
              case Crazyflie::ParamTypeInt32:
                broadcast_set_param<int32_t>(group, name, p.as_int());
                break;
              case Crazyflie::ParamTypeFloat:
                if (p.get_type() == rclcpp::PARAMETER_INTEGER) {
                  broadcast_set_param<float>(group, name, (float)p.as_int());
                } else {
                  broadcast_set_param<float>(group, name, p.as_double());
                }
                break;
              }
              break;
            }
          }
        } else {
          auto iter = crazyflies_.find(cfname);
          if (iter != crazyflies_.end()) {
            iter->second->change_parameter(p);
          }
        }
      }
    }
  }

  void on_watchdog_timer()
  {
    auto now = std::chrono::steady_clock::now();

    // motion capture
    // a) check if the rate was within specified bounds
    if (mocap_data_received_timepoints_.size() >= 2) {
      double mean_rate = 0;
      double min_rate = std::numeric_limits<double>::max();
      double max_rate = 0;
      int num_rates_wrong = 0;
      for (size_t i = 0; i < mocap_data_received_timepoints_.size() - 1; ++i) {
        std::chrono::duration<double> diff = mocap_data_received_timepoints_[i+1] - mocap_data_received_timepoints_[i];
        double rate = 1.0 / diff.count();
        mean_rate += rate;
        min_rate = std::min(min_rate, rate);
        max_rate = std::max(max_rate, rate);
        if (rate <= mocap_min_rate_ || rate >= mocap_max_rate_) {
          num_rates_wrong++;
        }
      }
      mean_rate /= (mocap_data_received_timepoints_.size() - 1);

      if (num_rates_wrong > 0) {
        RCLCPP_WARN(logger_, "[all] Motion capture rate off (#: %d, Avg: %.1f, Min: %.1f, Max: %.1f)", num_rates_wrong, mean_rate, min_rate, max_rate);
      }
    } else if (mocap_enabled_) {
      // b) warn if no data was received
      RCLCPP_WARN(logger_, "[all] Motion capture did not receive data!");
    }

    mocap_data_received_timepoints_.clear();

    if (publish_stats_) {

      crazyflie_interfaces::msg::ConnectionStatisticsArray msg;
      msg.header.stamp = this->get_clock()->now();
      msg.header.frame_id = "world"; // this is across broadcasters, which is not directly associated with single CFs; hence keep "world" here
      msg.stats.resize(broadcaster_.size());

      size_t i = 0;
      for (auto &bc : broadcaster_) {
        auto &cfbc = bc.second;

        auto stats = cfbc->connectionStatsDelta();

        msg.stats[i].uri = cfbc->uri();
        msg.stats[i].sent_count = stats.sent_count;
        msg.stats[i].sent_ping_count = stats.sent_ping_count;
        msg.stats[i].receive_count = stats.receive_count;
        msg.stats[i].enqueued_count = stats.enqueued_count;
        msg.stats[i].ack_count = stats.ack_count;
        ++i;
      }
      publisher_connection_stats_->publish(msg);
    }
  }

  template<class T>
  void broadcast_set_param(
    const std::string& group,
    const std::string& name,
    const T& value)
  {
    for (int i = 0; i < broadcasts_num_repeats_; ++i) {
      for (auto &bc : broadcaster_) {
        auto &cfbc = bc.second;
        cfbc->setParam<T>(group.c_str(), name.c_str(), value);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(broadcasts_delay_between_repeats_ms_));
    }
  }

  void update_name_to_id_map(const std::string& name, uint8_t id)
  {
    const auto iter = name_to_id_.find(name);
    if (iter != name_to_id_.end()) {
      RCLCPP_WARN(logger_, "[all] At least two objects with the same id (%d, %s, %s)", id, name.c_str(), iter->first.c_str());
    } else {
      name_to_id_.insert(std::make_pair(name, id));
    }
  }

  private:
    rclcpp::Logger logger_;

    // subscribers
    rclcpp::Subscription<crazyflie_interfaces::msg::FullState>::SharedPtr subscription_cmd_full_state_;
    rclcpp::Subscription<crazyflie_interfaces::msg::Hover>::SharedPtr subscription_cmd_hover_;
    rclcpp::Subscription<crazyflie_interfaces::msg::VelocityWorld>::SharedPtr subscription_cmd_velocity_world_;
    rclcpp::Subscription<NamedPoseArray>::SharedPtr sub_poses_;

    // services
    rclcpp::Service<Empty>::SharedPtr service_emergency_;
    rclcpp::Service<StartTrajectory>::SharedPtr service_start_trajectory_;
    rclcpp::Service<Takeoff>::SharedPtr service_takeoff_;
    rclcpp::Service<Land>::SharedPtr service_land_;
    rclcpp::Service<GoTo>::SharedPtr service_go_to_;
    rclcpp::Service<NotifySetpointsStop>::SharedPtr service_notify_setpoints_stop_;
    rclcpp::Service<Arm>::SharedPtr service_arm_;

    std::map<std::string, std::unique_ptr<CrazyflieROS>> crazyflies_;



    // broadcastUri -> broadcast object
    std::map<std::string, std::unique_ptr<CrazyflieBroadcaster>> broadcaster_;

    // maps CF name -> CF id
    std::map<std::string, uint8_t> name_to_id_;

    // global params
    int broadcasts_num_repeats_;
    int broadcasts_delay_between_repeats_ms_;

    // parameter updates
    std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
    std::shared_ptr<rclcpp::ParameterEventCallbackHandle> cb_handle_;

    // sanity checks
    rclcpp::TimerBase::SharedPtr watchdog_timer_;
    bool mocap_enabled_;
    float mocap_min_rate_;
    float mocap_max_rate_;
    std::vector<std::chrono::time_point<std::chrono::steady_clock>> mocap_data_received_timepoints_;
    bool publish_stats_;
    rclcpp::Publisher<crazyflie_interfaces::msg::ConnectionStatisticsArray>::SharedPtr publisher_connection_stats_;

    // multithreading
    rclcpp::CallbackGroup::SharedPtr callback_group_mocap_;
    rclcpp::CallbackGroup::SharedPtr callback_group_all_cmd_;
    rclcpp::CallbackGroup::SharedPtr callback_group_all_srv_;
    rclcpp::CallbackGroup::SharedPtr callback_group_cf_cmd_;
    rclcpp::CallbackGroup::SharedPtr callback_group_cf_srv_;
  };

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);


  auto node = std::make_shared<CrazyflieServer>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
