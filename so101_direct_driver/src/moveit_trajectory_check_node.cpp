#include <algorithm>
#include <cmath>
#include <cstddef>
#include <map>
#include <string>
#include <vector>

#include "moveit_msgs/msg/display_trajectory.hpp"
#include "rclcpp/rclcpp.hpp"

namespace so101_direct_driver
{

class MoveItTrajectoryCheckNode : public rclcpp::Node
{
public:
  MoveItTrajectoryCheckNode()
  : Node("so101_moveit_trajectory_check")
  {
    declare_parameter<std::vector<std::string>>(
      "joint_names",
      {"shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"});
    declare_parameter<std::vector<double>>(
      "joint_lower_limits",
      {-1.91986, -1.80, -1.69, -1.65806, -2.74385});
    declare_parameter<std::vector<double>>(
      "joint_upper_limits",
      {1.91986, 1.80, 1.69, 1.65806, 2.84121});
    declare_parameter<double>("max_step_rad", 0.20);
    declare_parameter<std::string>("trajectory_topic", "/display_planned_path");

    joint_names_ = get_parameter("joint_names").as_string_array();
    lower_limits_ = get_parameter("joint_lower_limits").as_double_array();
    upper_limits_ = get_parameter("joint_upper_limits").as_double_array();
    max_step_rad_ = get_parameter("max_step_rad").as_double();
    trajectory_topic_ = get_parameter("trajectory_topic").as_string();

    validate_parameters();

    for (size_t i = 0; i < joint_names_.size(); ++i) {
      joint_index_[joint_names_[i]] = i;
    }

    subscription_ = create_subscription<moveit_msgs::msg::DisplayTrajectory>(
      trajectory_topic_,
      rclcpp::QoS(10),
      std::bind(&MoveItTrajectoryCheckNode::trajectory_callback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "MoveIt 轨迹检查节点已启动，订阅：%s", trajectory_topic_.c_str());
    RCLCPP_INFO(get_logger(), "该节点只检查轨迹，不会控制机械臂。");
  }

private:
  void validate_parameters()
  {
    if (joint_names_.empty()) {
      throw std::runtime_error("joint_names 不能为空。");
    }
    if (lower_limits_.size() != joint_names_.size() || upper_limits_.size() != joint_names_.size()) {
      throw std::runtime_error("joint limits 数量必须和 joint_names 一致。");
    }
    if (max_step_rad_ <= 0.0) {
      throw std::runtime_error("max_step_rad 必须大于 0。");
    }
  }

  void trajectory_callback(const moveit_msgs::msg::DisplayTrajectory::SharedPtr msg)
  {
    if (msg->trajectory.empty()) {
      RCLCPP_WARN(get_logger(), "收到空的 MoveIt 轨迹。");
      return;
    }

    RCLCPP_WARN(get_logger(), "收到 MoveIt 规划结果，轨迹数量：%zu。", msg->trajectory.size());

    for (size_t trajectory_index = 0; trajectory_index < msg->trajectory.size(); ++trajectory_index) {
      const auto & trajectory = msg->trajectory[trajectory_index].joint_trajectory;
      RCLCPP_WARN(
        get_logger(),
        "检查第 %zu 条轨迹：关节数 %zu，点数 %zu。",
        trajectory_index,
        trajectory.joint_names.size(),
        trajectory.points.size());

      if (trajectory.points.empty()) {
        RCLCPP_WARN(get_logger(), "轨迹没有点，跳过。");
        continue;
      }

      check_joint_names(trajectory.joint_names);
      check_points(trajectory.joint_names, trajectory.points);
    }
  }

  void check_joint_names(const std::vector<std::string> & trajectory_joint_names)
  {
    for (const auto & required_joint : joint_names_) {
      const auto found = std::find(
        trajectory_joint_names.begin(), trajectory_joint_names.end(), required_joint);
      if (found == trajectory_joint_names.end()) {
        RCLCPP_ERROR(get_logger(), "轨迹缺少关节：%s", required_joint.c_str());
      }
    }
  }

  template<typename PointT>
  void check_points(
    const std::vector<std::string> & trajectory_joint_names,
    const std::vector<PointT> & points)
  {
    bool has_error = false;
    double max_observed_step = 0.0;

    for (size_t point_index = 0; point_index < points.size(); ++point_index) {
      const auto & point = points[point_index];
      if (point.positions.size() != trajectory_joint_names.size()) {
        RCLCPP_ERROR(
          get_logger(),
          "第 %zu 个点 positions 数量不等于 joint_names 数量。",
          point_index);
        has_error = true;
        continue;
      }

      for (size_t traj_joint_index = 0; traj_joint_index < trajectory_joint_names.size(); ++traj_joint_index) {
        const auto map_it = joint_index_.find(trajectory_joint_names[traj_joint_index]);
        if (map_it == joint_index_.end()) {
          continue;
        }

        const size_t checked_index = map_it->second;
        const double position = point.positions[traj_joint_index];
        if (position < lower_limits_[checked_index] || position > upper_limits_[checked_index]) {
          RCLCPP_ERROR(
            get_logger(),
            "第 %zu 个点关节 %s 超出限制：%.4f，不在 [%.4f, %.4f]。",
            point_index,
            trajectory_joint_names[traj_joint_index].c_str(),
            position,
            lower_limits_[checked_index],
            upper_limits_[checked_index]);
          has_error = true;
        }

        if (point_index > 0) {
          const auto & previous = points[point_index - 1];
          if (previous.positions.size() == trajectory_joint_names.size()) {
            const double step = std::abs(position - previous.positions[traj_joint_index]);
            max_observed_step = std::max(max_observed_step, step);
            if (step > max_step_rad_) {
              RCLCPP_ERROR(
                get_logger(),
                "第 %zu -> %zu 个点关节 %s 单步变化过大：%.4f rad，限制 %.4f rad。",
                point_index - 1,
                point_index,
                trajectory_joint_names[traj_joint_index].c_str(),
                step,
                max_step_rad_);
              has_error = true;
            }
          }
        }
      }
    }

    const auto & first = points.front();
    const auto & last = points.back();
    RCLCPP_WARN(get_logger(), "轨迹起点：%s", format_point(trajectory_joint_names, first.positions).c_str());
    RCLCPP_WARN(get_logger(), "轨迹终点：%s", format_point(trajectory_joint_names, last.positions).c_str());
    RCLCPP_WARN(get_logger(), "最大单步变化：%.4f rad。", max_observed_step);

    if (has_error) {
      RCLCPP_ERROR(get_logger(), "轨迹检查未通过。不要执行到真机。");
    } else {
      RCLCPP_WARN(get_logger(), "轨迹检查通过。该节点仍然不会执行真机。");
    }
  }

  std::string format_point(
    const std::vector<std::string> & trajectory_joint_names,
    const std::vector<double> & positions) const
  {
    std::string output;
    for (size_t i = 0; i < trajectory_joint_names.size() && i < positions.size(); ++i) {
      if (i > 0) {
        output += ", ";
      }
      output += trajectory_joint_names[i] + "=" + format_double(positions[i]);
    }
    return output;
  }

  std::string format_double(double value) const
  {
    char buffer[64];
    std::snprintf(buffer, sizeof(buffer), "%.4f", value);
    return std::string(buffer);
  }

  std::vector<std::string> joint_names_;
  std::vector<double> lower_limits_;
  std::vector<double> upper_limits_;
  double max_step_rad_ {0.20};
  std::string trajectory_topic_;
  std::map<std::string, size_t> joint_index_;

  rclcpp::Subscription<moveit_msgs::msg::DisplayTrajectory>::SharedPtr subscription_;
};

}  // namespace so101_direct_driver

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<so101_direct_driver::MoveItTrajectoryCheckNode>());
  } catch (const std::exception & error) {
    std::cerr << "MoveIt 轨迹检查节点启动失败：" << error.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
