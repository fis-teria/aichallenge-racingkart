// Copyright 2023 Tier IV, Inc. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "simple_trajectory_generator/execution_profile.hpp"
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <cmath>
#include <exception>
#include <filesystem>
#include <fstream>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <string>
#include <vector>

using Trajectory = autoware_auto_planning_msgs::msg::Trajectory;
using TrajectoryPoint = autoware_auto_planning_msgs::msg::TrajectoryPoint;

class CSVToTrajectory : public rclcpp::Node {
public:
  CSVToTrajectory() : Node("csv_to_trajectory_node") {
    const auto rb_qos =
        rclcpp::QoS(rclcpp::KeepLast(1)).durability_volatile().best_effort();
    pub_ = this->create_publisher<Trajectory>("trajectory", rb_qos);

    declare_parameter("csv_path", "");
    z_ = declare_parameter<float>("z");
    execution_profile_config_.max_speed_mps =
        declare_parameter<double>("execution_profile.max_speed_mps", 10.0);
    execution_profile_config_.max_arc_spacing_m =
        declare_parameter<double>("execution_profile.max_arc_spacing_m", 0.25);
    execution_profile_config_.max_yaw_step_rad =
        declare_parameter<double>("execution_profile.max_yaw_step_rad", 0.05);
    std::string csv_path = get_parameter("csv_path").as_string();

    if (csv_path.empty()) {
      RCLCPP_ERROR(get_logger(), "CSV path is not specified");
      return;
    }

    if (!loadCSVTrajectory(csv_path)) {
      RCLCPP_ERROR(get_logger(), "Failed to load CSV file: %s",
                   csv_path.c_str());
      return;
    }
    current_csv_path_ = csv_path;

    RCLCPP_INFO(get_logger(), "Loaded trajectory from CSV with %zu points",
                csv_trajectory_.points.size());

    timer_ = rclcpp::create_timer(
        this, get_clock(), std::chrono::seconds(1),
        std::bind(&CSVToTrajectory::publish_trajectory, this));
    // Launch overrides are startup configuration. Register the immutability
    // guard only after they have been declared, validated, and loaded.
    set_parameter_callback_handle_ =
        this->add_on_set_parameters_callback(std::bind(
            &CSVToTrajectory::on_parameter_event, this, std::placeholders::_1));
  }

private:
  enum class CSVFormat {
    kPoseWithQuaternion,
    kReferencePath,
  };

  static geometry_msgs::msg::Quaternion
  createQuaternionFromYaw(const double yaw_rad) {
    geometry_msgs::msg::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = std::sin(yaw_rad * 0.5);
    quaternion.w = std::cos(yaw_rad * 0.5);
    return quaternion;
  }

  static std::vector<std::string> splitCSVLine(const std::string &line) {
    std::stringstream ss(line);
    std::string token;
    std::vector<std::string> tokens;
    while (std::getline(ss, token, ',')) {
      if (!token.empty() && token.back() == '\r') {
        token.pop_back();
      }
      tokens.push_back(token);
    }
    return tokens;
  }

  static std::vector<double> parseCSVValues(const std::string &line) {
    const auto tokens = splitCSVLine(line);
    std::vector<double> values;
    values.reserve(tokens.size());
    for (const auto &token : tokens) {
      values.push_back(std::stod(token));
    }
    return values;
  }

  static bool isBlankLine(const std::string &line) {
    return line.find_first_not_of(" \t\r\n") == std::string::npos;
  }

  static CSVFormat detectCSVFormat(const std::string &header_line) {
    const auto header = splitCSVLine(header_line);
    if (header.size() >= 7 && header[0] == "s_m" && header[1] == "x_m" &&
        header[2] == "y_m" && header[3] == "psi_rad" && header[5] == "vx_mps") {
      return CSVFormat::kReferencePath;
    }
    return CSVFormat::kPoseWithQuaternion;
  }

  bool loadCSVTrajectory(const std::string &csv_path) {
    std::ifstream file(csv_path);
    if (!file.is_open()) {
      return false;
    }

    std::string line;
    std::getline(file, line);
    const CSVFormat csv_format = detectCSVFormat(line);

    Trajectory source_trajectory;
    source_trajectory.header.stamp = this->now();
    source_trajectory.header.frame_id = "map";

    while (std::getline(file, line)) {
      if (isBlankLine(line)) {
        continue;
      }

      std::vector<double> values;
      try {
        values = parseCSVValues(line);
      } catch (const std::exception &exception) {
        RCLCPP_ERROR(get_logger(),
                     "Invalid numeric value in CSV; rejecting complete "
                     "trajectory: %s",
                     exception.what());
        return false;
      }

      if (csv_format == CSVFormat::kPoseWithQuaternion && values.size() != 8) {
        RCLCPP_ERROR(get_logger(),
                     "Invalid pose CSV line format; rejecting complete "
                     "trajectory (expected 8 values)");
        return false;
      }
      if (csv_format == CSVFormat::kReferencePath && values.size() < 7) {
        RCLCPP_ERROR(get_logger(),
                     "Invalid reference path CSV line format; rejecting "
                     "complete trajectory (expected at least 7 values)");
        return false;
      }

      TrajectoryPoint point;
      point.pose.position.z = z_;
      if (csv_format == CSVFormat::kReferencePath) {
        point.pose.position.x = values[1];
        point.pose.position.y = values[2];
        point.pose.orientation = createQuaternionFromYaw(values[3]);
        point.longitudinal_velocity_mps = values[5];
        point.acceleration_mps2 = values[6];
      } else {
        point.pose.position.x = values[0];
        point.pose.position.y = values[1];
        point.pose.orientation.x = values[3];
        point.pose.orientation.y = values[4];
        point.pose.orientation.z = values[5];
        point.pose.orientation.w = values[6];
        point.longitudinal_velocity_mps = values[7];
        point.acceleration_mps2 = 0.0;
      }

      point.lateral_velocity_mps = 0.0;
      point.heading_rate_rps = 0.0;

      source_trajectory.points.push_back(point);
    }

    const auto profile = simple_trajectory_generator::buildExecutionProfile(
        source_trajectory, execution_profile_config_);
    if (!profile.valid) {
      RCLCPP_ERROR(get_logger(), "Execution profile rejected CSV: %s",
                   profile.reason.c_str());
      return false;
    }
    csv_trajectory_ = profile.trajectory;
    RCLCPP_INFO(get_logger(),
                "Built bounded execution profile: source_points=%zu "
                "output_points=%zu speed=%.3f m/s",
                source_trajectory.points.size(), csv_trajectory_.points.size(),
                profile.execution_speed_mps);
    return true;
  }

  void publish_trajectory() {
    if (csv_trajectory_.points.empty()) {
      RCLCPP_WARN(get_logger(), "No trajectory points to publish");
      return;
    }

    csv_trajectory_.header.stamp = this->now();
    pub_->publish(csv_trajectory_);
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 60000 /*ms*/,
                         "Published trajectory with %zu points",
                         csv_trajectory_.points.size());
  }

  rcl_interfaces::msg::SetParametersResult
  on_parameter_event(const std::vector<rclcpp::Parameter> &parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "";

    for (const auto &param : parameters) {
      if (param.get_name() == "csv_path") {
        if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
          std::string new_csv_path = param.as_string();
          // new_csv_pathがFileSystemのパスであることを確認
          if (!std::filesystem::exists(new_csv_path)) {
            RCLCPP_ERROR(get_logger(), "File does not exist: '%s'",
                         new_csv_path.c_str());
            result.successful = false;
            result.reason = "File does not exist.";
            continue;
          }

          if (new_csv_path != current_csv_path_) {
            RCLCPP_INFO(get_logger(),
                        "csv_path parameter changed from '%s' to '%s'",
                        current_csv_path_.c_str(), new_csv_path.c_str());

            // 新しいCSVファイルの読み込みを試みる
            if (loadCSVTrajectory(new_csv_path)) {
              current_csv_path_ = new_csv_path;
              RCLCPP_INFO(get_logger(),
                          "Successfully loaded new trajectory from CSV: %s "
                          "with %zu points",
                          current_csv_path_.c_str(),
                          csv_trajectory_.points.size());
            } else {
              RCLCPP_ERROR(
                  get_logger(),
                  "Failed to load new CSV file: %s. Keeping old trajectory.",
                  new_csv_path.c_str());
              result.successful = false;
              result.reason = "Failed to load new CSV file.";
            }
          }
        } else {
          RCLCPP_WARN(get_logger(), "Parameter 'csv_path' received with wrong "
                                    "type. Expected string.");
          result.successful = false;
          result.reason = "Invalid type for csv_path parameter.";
        }
      } else if (param.get_name() == "z") {
        if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE ||
            param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
          z_ = static_cast<float>(param.as_double());
          RCLCPP_INFO(get_logger(), "z parameter changed to %f", z_);
        } else {
          RCLCPP_WARN(
              get_logger(),
              "Parameter 'z' received with wrong type. Expected float/double.");
          result.successful = false;
          result.reason = "Invalid type for z parameter.";
        }
      } else if (param.get_name().rfind("execution_profile.", 0U) == 0U) {
        result.successful = false;
        result.reason =
            "Execution profile parameters are immutable after startup.";
      }
    }
    return result;
  }

  rclcpp::Publisher<Trajectory>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  Trajectory csv_trajectory_;
  float z_;
  simple_trajectory_generator::ExecutionProfileConfig execution_profile_config_;
  std::string current_csv_path_;
  OnSetParametersCallbackHandle::SharedPtr set_parameter_callback_handle_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CSVToTrajectory>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
