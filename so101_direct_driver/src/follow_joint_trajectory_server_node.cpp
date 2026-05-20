#include <fcntl.h>
#include <cerrno>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace so101_direct_driver
{

class FeetechBus
{
public:
  FeetechBus(const std::string & port, int baudrate, double timeout_seconds)
  : timeout_seconds_(timeout_seconds)
  {
    fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd_ < 0) {
      throw std::runtime_error("打开串口失败：" + std::string(std::strerror(errno)));
    }
    try {
      configure_port(baudrate);
      tcflush(fd_, TCIOFLUSH);
    } catch (...) {
      close();
      throw;
    }
  }

  ~FeetechBus()
  {
    close();
  }

  bool ping(uint8_t motor_id)
  {
    send_packet(motor_id, instruction_ping_, {});
    return read_status_packet(motor_id).first;
  }

  uint16_t read_u16(uint8_t motor_id, uint8_t address)
  {
    std::lock_guard<std::mutex> guard(io_mutex_);
    send_packet(motor_id, instruction_read_, {address, 2});
    auto packet = read_status_packet(motor_id);
    if (!packet.first) {
      throw std::runtime_error("读取 " + std::to_string(motor_id) + " 号舵机超时。");
    }
    const auto & payload = packet.second;
    if (payload.empty() || payload[0] != 0 || payload.size() < 3) {
      throw std::runtime_error("读取 " + std::to_string(motor_id) + " 号舵机失败。");
    }
    return static_cast<uint16_t>(payload[1]) | static_cast<uint16_t>(payload[2] << 8);
  }

  void write_u8(uint8_t motor_id, uint8_t address, uint8_t value)
  {
    std::lock_guard<std::mutex> guard(io_mutex_);
    send_packet(motor_id, instruction_write_, {address, value});
    check_write_response(motor_id);
  }

  void write_u16(uint8_t motor_id, uint8_t address, uint16_t value)
  {
    std::lock_guard<std::mutex> guard(io_mutex_);
    send_packet(
      motor_id,
      instruction_write_,
      {address, static_cast<uint8_t>(value & 0xff), static_cast<uint8_t>((value >> 8) & 0xff)});
    check_write_response(motor_id);
  }

  void close()
  {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }

private:
  void configure_port(int baudrate)
  {
    termios tty {};
    if (tcgetattr(fd_, &tty) != 0) {
      throw std::runtime_error("读取串口配置失败：" + std::string(std::strerror(errno)));
    }
    cfmakeraw(&tty);
    tty.c_cflag |= static_cast<tcflag_t>(CLOCAL | CREAD);
    tty.c_cflag &= static_cast<tcflag_t>(~CRTSCTS);
    tty.c_cflag &= static_cast<tcflag_t>(~CSTOPB);
    tty.c_cflag &= static_cast<tcflag_t>(~PARENB);
    tty.c_cflag &= static_cast<tcflag_t>(~CSIZE);
    tty.c_cflag |= CS8;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;
    const speed_t speed = baud_to_termios(baudrate);
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);
    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
      throw std::runtime_error("写入串口配置失败：" + std::string(std::strerror(errno)));
    }
  }

  speed_t baud_to_termios(int baudrate) const
  {
    switch (baudrate) {
      case 9600:
        return B9600;
      case 57600:
        return B57600;
      case 115200:
        return B115200;
      case 1000000:
        return B1000000;
      default:
        throw std::runtime_error("不支持该波特率：" + std::to_string(baudrate));
    }
  }

  void check_write_response(uint8_t motor_id)
  {
    auto packet = read_status_packet(motor_id);
    if (!packet.first) {
      throw std::runtime_error("写入 " + std::to_string(motor_id) + " 号舵机超时。");
    }
    if (!packet.second.empty() && packet.second[0] != 0) {
      throw std::runtime_error("写入 " + std::to_string(motor_id) + " 号舵机失败。");
    }
  }

  void send_packet(uint8_t motor_id, uint8_t instruction, const std::vector<uint8_t> & params)
  {
    std::vector<uint8_t> packet;
    packet.reserve(params.size() + 6);
    packet.push_back(0xff);
    packet.push_back(0xff);
    packet.push_back(motor_id);
    packet.push_back(static_cast<uint8_t>(params.size() + 2));
    packet.push_back(instruction);
    packet.insert(packet.end(), params.begin(), params.end());
    uint16_t sum = 0;
    for (size_t i = 2; i < packet.size(); ++i) {
      sum += packet[i];
    }
    packet.push_back(static_cast<uint8_t>((~sum) & 0xff));
    const ssize_t written = ::write(fd_, packet.data(), packet.size());
    if (written != static_cast<ssize_t>(packet.size())) {
      throw std::runtime_error("写入串口数据失败。");
    }
    tcdrain(fd_);
  }

  std::pair<bool, std::vector<uint8_t>> read_status_packet(uint8_t expected_id)
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(timeout_seconds_));
    while (std::chrono::steady_clock::now() < deadline) {
      uint8_t first = 0;
      if (!read_byte(first, deadline)) {
        return {false, {}};
      }
      if (first != 0xff) {
        continue;
      }
      uint8_t second = 0;
      if (!read_byte(second, deadline) || second != 0xff) {
        continue;
      }
      uint8_t motor_id = 0;
      uint8_t length = 0;
      if (!read_byte(motor_id, deadline) || !read_byte(length, deadline)) {
        return {false, {}};
      }
      std::vector<uint8_t> payload(length);
      for (auto & value : payload) {
        if (!read_byte(value, deadline)) {
          return {false, {}};
        }
      }
      if (payload.empty()) {
        continue;
      }
      const uint8_t checksum = payload.back();
      uint16_t sum = motor_id + length;
      for (size_t i = 0; i + 1 < payload.size(); ++i) {
        sum += payload[i];
      }
      if (checksum != static_cast<uint8_t>((~sum) & 0xff) || motor_id != expected_id) {
        continue;
      }
      payload.pop_back();
      return {true, payload};
    }
    return {false, {}};
  }

  bool read_byte(uint8_t & value, const std::chrono::steady_clock::time_point & deadline)
  {
    while (std::chrono::steady_clock::now() < deadline) {
      fd_set read_fds;
      FD_ZERO(&read_fds);
      FD_SET(fd_, &read_fds);
      const auto remaining = deadline - std::chrono::steady_clock::now();
      const auto remaining_us = std::chrono::duration_cast<std::chrono::microseconds>(remaining).count();
      timeval timeout {};
      timeout.tv_sec = static_cast<time_t>(remaining_us / 1000000);
      timeout.tv_usec = static_cast<suseconds_t>(remaining_us % 1000000);
      const int result = select(fd_ + 1, &read_fds, nullptr, nullptr, &timeout);
      if (result <= 0) {
        return false;
      }
      const ssize_t count = ::read(fd_, &value, 1);
      if (count == 1) {
        return true;
      }
    }
    return false;
  }

  int fd_ {-1};
  double timeout_seconds_ {0.2};
  std::mutex io_mutex_;
  static constexpr uint8_t instruction_ping_ = 0x01;
  static constexpr uint8_t instruction_read_ = 0x02;
  static constexpr uint8_t instruction_write_ = 0x03;
};

class FollowJointTrajectoryServerNode : public rclcpp::Node
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandle = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

  FollowJointTrajectoryServerNode()
  : Node("so101_follow_joint_trajectory_server")
  {
    declare_parameters();
    read_parameters();
    validate_config();

    bus_ = std::make_unique<FeetechBus>(port_, baudrate_, response_timeout_);
    ping_motors();

    joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / read_rate_)),
      std::bind(&FollowJointTrajectoryServerNode::publish_positions, this));

    action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
      this,
      "follower_arm_controller/follow_joint_trajectory",
      std::bind(&FollowJointTrajectoryServerNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&FollowJointTrajectoryServerNode::handle_cancel, this, std::placeholders::_1),
      std::bind(&FollowJointTrajectoryServerNode::handle_accepted, this, std::placeholders::_1));

    RCLCPP_WARN(get_logger(), "FollowJointTrajectory action server 已启动。");
    RCLCPP_WARN(get_logger(), "Action: follower_arm_controller/follow_joint_trajectory");
  }

  ~FollowJointTrajectoryServerNode() override
  {
    cancel_requested_.store(true);
    if (execute_thread_.joinable()) {
      execute_thread_.join();
    }
    if (bus_) {
      bus_->close();
    }
  }

private:
  void declare_parameters()
  {
    declare_parameter<std::string>("port", "/dev/ttyACM0");
    declare_parameter<int>("baudrate", 1000000);
    declare_parameter<double>("read_rate", 10.0);
    declare_parameter<int>("present_position_address", 56);
    declare_parameter<int>("goal_position_address", 42);
    declare_parameter<int>("torque_enable_address", 40);
    declare_parameter<double>("position_ticks_per_turn", 4096.0);
    declare_parameter<double>("response_timeout", 0.2);
    declare_parameter<double>("time_scale", 5.0);
    declare_parameter<double>("max_step_rad", 0.20);
    declare_parameter<double>("max_velocity_rad_s", 0.25);
    declare_parameter<double>("command_rate", 25.0);
    declare_parameter<int>("command_deadband_ticks", 2);
    declare_parameter<std::vector<std::string>>(
      "joint_names",
      {"shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"});
    declare_parameter<std::vector<int64_t>>("motor_ids", {1, 2, 3, 4, 5, 6});
    declare_parameter<std::vector<int64_t>>("range_mins", {0, 0, 0, 0, 0, 0});
    declare_parameter<std::vector<int64_t>>("range_maxs", {4095, 4095, 4095, 4095, 4095, 4095});
    declare_parameter<std::vector<int64_t>>("signs", {1, 1, 1, 1, 1, 1});
    declare_parameter<std::vector<int64_t>>("zero_positions", {2048, 2048, 2048, 2048, 2048, 2048});
    declare_parameter<std::vector<std::string>>(
      "controlled_joints",
      {"shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"});
  }

  void read_parameters()
  {
    port_ = get_parameter("port").as_string();
    baudrate_ = static_cast<int>(get_parameter("baudrate").as_int());
    read_rate_ = get_parameter("read_rate").as_double();
    present_position_address_ = static_cast<uint8_t>(get_parameter("present_position_address").as_int());
    goal_position_address_ = static_cast<uint8_t>(get_parameter("goal_position_address").as_int());
    torque_enable_address_ = static_cast<uint8_t>(get_parameter("torque_enable_address").as_int());
    ticks_per_turn_ = get_parameter("position_ticks_per_turn").as_double();
    response_timeout_ = get_parameter("response_timeout").as_double();
    time_scale_ = get_parameter("time_scale").as_double();
    max_step_rad_ = get_parameter("max_step_rad").as_double();
    max_velocity_rad_s_ = get_parameter("max_velocity_rad_s").as_double();
    command_rate_ = get_parameter("command_rate").as_double();
    command_deadband_ticks_ = static_cast<int>(get_parameter("command_deadband_ticks").as_int());
    joint_names_ = get_parameter("joint_names").as_string_array();
    motor_ids_ = to_int_vector(get_parameter("motor_ids").as_integer_array());
    range_mins_ = to_int_vector(get_parameter("range_mins").as_integer_array());
    range_maxs_ = to_int_vector(get_parameter("range_maxs").as_integer_array());
    signs_ = to_int_vector(get_parameter("signs").as_integer_array());
    zero_positions_ = to_int_vector(get_parameter("zero_positions").as_integer_array());
    controlled_joints_ = get_parameter("controlled_joints").as_string_array();
  }

  std::vector<int> to_int_vector(const std::vector<int64_t> & values)
  {
    std::vector<int> result;
    for (const auto value : values) {
      result.push_back(static_cast<int>(value));
    }
    return result;
  }

  void validate_config()
  {
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      joint_index_[joint_names_[i]] = i;
    }
    cached_command_positions_.assign(joint_names_.size(), 0.0);
    last_command_raw_.assign(joint_names_.size(), std::numeric_limits<int>::min());
  }

  void ping_motors()
  {
    for (const auto & joint : controlled_joints_) {
      const auto index = joint_index_.at(joint);
      if (!bus_->ping(static_cast<uint8_t>(motor_ids_[index]))) {
        throw std::runtime_error(joint + " 舵机没有响应。");
      }
      RCLCPP_INFO(get_logger(), "%s：舵机已响应。", joint.c_str());
    }
  }

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const FollowJointTrajectory::Goal> goal)
  {
    if (executing_.load()) {
      RCLCPP_ERROR(get_logger(), "已有轨迹正在执行，拒绝新目标。");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (!validate_goal(*goal)) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    RCLCPP_WARN(get_logger(), "接受 MoveIt 轨迹目标，点数：%zu。", goal->trajectory.points.size());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle>)
  {
    cancel_requested_.store(true);
    RCLCPP_WARN(get_logger(), "收到轨迹取消请求。");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
  {
    cancel_requested_.store(false);
    executing_.store(true);
    if (execute_thread_.joinable()) {
      execute_thread_.join();
    }
    execute_thread_ = std::thread([this, goal_handle]() { execute_goal(goal_handle); });
  }

  bool validate_goal(const FollowJointTrajectory::Goal & goal)
  {
    if (goal.trajectory.points.empty()) {
      RCLCPP_ERROR(get_logger(), "轨迹为空。");
      return false;
    }
    for (const auto & joint : controlled_joints_) {
      if (std::find(goal.trajectory.joint_names.begin(), goal.trajectory.joint_names.end(), joint) ==
        goal.trajectory.joint_names.end())
      {
        RCLCPP_ERROR(get_logger(), "轨迹缺少关节：%s", joint.c_str());
        return false;
      }
    }
    return true;
  }

  void execute_goal(const std::shared_ptr<GoalHandle> goal_handle)
  {
    auto result = std::make_shared<FollowJointTrajectory::Result>();
    const auto goal = goal_handle->get_goal();

    try {
      enable_torque();
      const std::vector<double> current_positions = read_current_positions();
      cache_command_positions(current_positions);
      std::vector<double> previous_positions;
      double previous_trajectory_time = 0.0;
      double previous_execution_time = 0.0;
      const auto start = std::chrono::steady_clock::now();

      for (const auto & point : goal->trajectory.points) {
        if (cancel_requested_.load()) {
          result->error_code = FollowJointTrajectory::Result::PATH_TOLERANCE_VIOLATED;
          result->error_string = "轨迹已取消。";
          goal_handle->canceled(result);
          executing_.store(false);
          return;
        }

        validate_point(goal->trajectory.joint_names, point);
        if (previous_positions.empty()) {
          previous_positions = positions_in_trajectory_order(goal->trajectory.joint_names, current_positions, point.positions);
        }
        const double trajectory_time = duration_to_seconds(point.time_from_start);
        const double execution_time = calculate_segment_end_time(
          goal->trajectory.joint_names,
          previous_positions,
          point,
          previous_trajectory_time,
          previous_execution_time,
          trajectory_time);
        execute_segment(
          goal->trajectory.joint_names,
          previous_positions,
          point,
          previous_execution_time,
          execution_time,
          start);
        previous_positions = point.positions;
        previous_trajectory_time = trajectory_time;
        previous_execution_time = execution_time;
      }

      result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
      result->error_string = "轨迹执行完成。";
      goal_handle->succeed(result);
      RCLCPP_WARN(get_logger(), "MoveIt 轨迹执行完成。");
    } catch (const std::exception & error) {
      result->error_code = FollowJointTrajectory::Result::INVALID_JOINTS;
      result->error_string = error.what();
      goal_handle->abort(result);
      RCLCPP_ERROR(get_logger(), "轨迹执行失败：%s", error.what());
    }

    executing_.store(false);
    reset_command_cache();
  }

  void enable_torque()
  {
    for (const auto & joint : controlled_joints_) {
      const auto index = joint_index_.at(joint);
      bus_->write_u8(static_cast<uint8_t>(motor_ids_[index]), torque_enable_address_, 1);
    }
    RCLCPP_WARN(get_logger(), "已使能 arm 关节扭矩。");
  }

  void validate_point(
    const std::vector<std::string> & trajectory_joints,
    const trajectory_msgs::msg::JointTrajectoryPoint & point)
  {
    if (point.positions.size() != trajectory_joints.size()) {
      throw std::runtime_error("轨迹点 positions 数量不匹配。");
    }
    for (size_t i = 0; i < trajectory_joints.size(); ++i) {
      const auto internal = joint_index_.find(trajectory_joints[i]);
      if (internal == joint_index_.end()) {
        continue;
      }
      const int raw = radians_to_raw(point.positions[i], internal->second);
      if (raw < range_mins_[internal->second] || raw > range_maxs_[internal->second]) {
        throw std::runtime_error("轨迹点超出舵机范围：" + trajectory_joints[i]);
      }
    }
  }

  double calculate_segment_end_time(
    const std::vector<std::string> & trajectory_joints,
    const std::vector<double> & previous_positions,
    const trajectory_msgs::msg::JointTrajectoryPoint & point,
    double previous_trajectory_time,
    double previous_execution_time,
    double trajectory_time) const
  {
    const double scaled_duration = std::max(0.0, trajectory_time - previous_trajectory_time) * time_scale_;
    double velocity_limited_duration = 0.0;

    if (!previous_positions.empty() && max_velocity_rad_s_ > 0.0) {
      double max_delta = 0.0;
      for (const auto & joint : controlled_joints_) {
        const auto traj_it = std::find(trajectory_joints.begin(), trajectory_joints.end(), joint);
        const size_t traj_index = static_cast<size_t>(std::distance(trajectory_joints.begin(), traj_it));
        max_delta = std::max(max_delta, std::abs(point.positions[traj_index] - previous_positions[traj_index]));
      }
      velocity_limited_duration = max_delta / max_velocity_rad_s_;
    }

    return previous_execution_time + std::max(scaled_duration, velocity_limited_duration);
  }

  void execute_segment(
    const std::vector<std::string> & trajectory_joints,
    const std::vector<double> & previous_positions,
    const trajectory_msgs::msg::JointTrajectoryPoint & point,
    double previous_time,
    double point_time,
    const std::chrono::steady_clock::time_point & start)
  {
    if (previous_positions.empty()) {
      sleep_until_time(start, point_time);
      write_positions(trajectory_joints, point.positions, true);
      return;
    }

    double max_delta = 0.0;
    for (const auto & joint : controlled_joints_) {
      const auto traj_it = std::find(trajectory_joints.begin(), trajectory_joints.end(), joint);
      const size_t traj_index = static_cast<size_t>(std::distance(trajectory_joints.begin(), traj_it));
      max_delta = std::max(max_delta, std::abs(point.positions[traj_index] - previous_positions[traj_index]));
    }

    const double duration = std::max(0.0, point_time - previous_time);
    const double command_period = 1.0 / std::max(command_rate_, 1.0);
    const size_t step_count = std::max<size_t>(
      1,
      std::max<size_t>(
        static_cast<size_t>(std::ceil(duration / command_period)),
        static_cast<size_t>(std::ceil(max_delta / std::max(max_step_rad_, 1e-6)))));

    for (size_t step = 1; step <= step_count; ++step) {
      if (cancel_requested_.load()) {
        return;
      }
      const double ratio = static_cast<double>(step) / static_cast<double>(step_count);
      std::vector<double> interpolated = point.positions;
      for (size_t i = 0; i < interpolated.size(); ++i) {
        interpolated[i] = previous_positions[i] + (point.positions[i] - previous_positions[i]) * ratio;
      }
      const double command_time = previous_time + (point_time - previous_time) * ratio;
      sleep_until_time(start, command_time);
      write_positions(trajectory_joints, interpolated, step == step_count);
    }
  }

  void write_point(
    const std::vector<std::string> & trajectory_joints,
    const trajectory_msgs::msg::JointTrajectoryPoint & point)
  {
    write_positions(trajectory_joints, point.positions, true);
  }

  void write_positions(
    const std::vector<std::string> & trajectory_joints,
    const std::vector<double> & positions,
    bool force)
  {
    for (const auto & joint : controlled_joints_) {
      const auto traj_it = std::find(trajectory_joints.begin(), trajectory_joints.end(), joint);
      const size_t traj_index = static_cast<size_t>(std::distance(trajectory_joints.begin(), traj_it));
      const size_t internal_index = joint_index_.at(joint);
      const int raw = radians_to_raw(positions[traj_index], internal_index);
      if (!force && last_command_raw_[internal_index] != std::numeric_limits<int>::min() &&
        std::abs(raw - last_command_raw_[internal_index]) < command_deadband_ticks_)
      {
        continue;
      }
      bus_->write_u16(
        static_cast<uint8_t>(motor_ids_[internal_index]),
        goal_position_address_,
        static_cast<uint16_t>(raw));
      last_command_raw_[internal_index] = raw;
      cache_command_position(internal_index, positions[traj_index]);
    }
  }

  std::vector<double> read_current_positions()
  {
    std::vector<double> positions(joint_names_.size(), 0.0);
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      const int raw = static_cast<int>(bus_->read_u16(static_cast<uint8_t>(motor_ids_[i]), present_position_address_));
      positions[i] = raw_to_radians(raw, i);
    }
    return positions;
  }

  std::vector<double> positions_in_trajectory_order(
    const std::vector<std::string> & trajectory_joints,
    const std::vector<double> & current_positions,
    const std::vector<double> & fallback_positions) const
  {
    std::vector<double> positions = fallback_positions;
    for (size_t i = 0; i < trajectory_joints.size(); ++i) {
      const auto internal = joint_index_.find(trajectory_joints[i]);
      if (internal != joint_index_.end()) {
        positions[i] = current_positions[internal->second];
      }
    }
    return positions;
  }

  void cache_command_positions(const std::vector<double> & positions)
  {
    std::lock_guard<std::mutex> guard(command_mutex_);
    cached_command_positions_ = positions;
  }

  void cache_command_position(size_t index, double position)
  {
    std::lock_guard<std::mutex> guard(command_mutex_);
    if (index < cached_command_positions_.size()) {
      cached_command_positions_[index] = position;
    }
  }

  void reset_command_cache()
  {
    std::lock_guard<std::mutex> guard(command_mutex_);
    std::fill(last_command_raw_.begin(), last_command_raw_.end(), std::numeric_limits<int>::min());
  }

  void sleep_until_time(
    const std::chrono::steady_clock::time_point & start,
    double seconds)
  {
    const auto target = start +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(seconds));
    std::this_thread::sleep_until(target);
  }

  double duration_to_seconds(const builtin_interfaces::msg::Duration & duration) const
  {
    return static_cast<double>(duration.sec) + static_cast<double>(duration.nanosec) * 1e-9;
  }

  int radians_to_raw(double radians, size_t index) const
  {
    return static_cast<int>(
      std::llround(
        static_cast<double>(zero_positions_[index]) +
        static_cast<double>(signs_[index]) * radians * ticks_per_turn_ / (2.0 * pi_)));
  }

  double raw_to_radians(int raw, size_t index) const
  {
    return static_cast<double>(signs_[index]) *
      (static_cast<double>(raw - zero_positions_[index]) / ticks_per_turn_) * 2.0 * pi_;
  }

  void publish_positions()
  {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = get_clock()->now();
    msg.name = joint_names_;
    msg.velocity.assign(joint_names_.size(), 0.0);
    if (executing_.load()) {
      std::lock_guard<std::mutex> guard(command_mutex_);
      if (cached_command_positions_.size() == joint_names_.size()) {
        msg.position = cached_command_positions_;
        joint_state_pub_->publish(msg);
        return;
      }
    }
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      try {
        const int raw = static_cast<int>(bus_->read_u16(static_cast<uint8_t>(motor_ids_[i]), present_position_address_));
        msg.position.push_back(raw_to_radians(raw, i));
      } catch (const std::exception & error) {
        RCLCPP_ERROR(get_logger(), "%s：读取位置失败：%s", joint_names_[i].c_str(), error.what());
        return;
      }
    }
    joint_state_pub_->publish(msg);
  }

  std::string port_;
  int baudrate_ {1000000};
  double read_rate_ {10.0};
  uint8_t present_position_address_ {56};
  uint8_t goal_position_address_ {42};
  uint8_t torque_enable_address_ {40};
  double ticks_per_turn_ {4096.0};
  double response_timeout_ {0.2};
  double time_scale_ {5.0};
  double max_step_rad_ {0.20};
  double max_velocity_rad_s_ {0.25};
  double command_rate_ {25.0};
  int command_deadband_ticks_ {2};
  std::vector<std::string> joint_names_;
  std::vector<std::string> controlled_joints_;
  std::vector<int> motor_ids_;
  std::vector<int> range_mins_;
  std::vector<int> range_maxs_;
  std::vector<int> signs_;
  std::vector<int> zero_positions_;
  std::map<std::string, size_t> joint_index_;
  std::vector<double> cached_command_positions_;
  std::vector<int> last_command_raw_;
  std::mutex command_mutex_;
  std::unique_ptr<FeetechBus> bus_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp_action::Server<FollowJointTrajectory>::SharedPtr action_server_;
  std::atomic<bool> executing_ {false};
  std::atomic<bool> cancel_requested_ {false};
  std::thread execute_thread_;
  static constexpr double pi_ = 3.14159265358979323846;
};

}  // namespace so101_direct_driver

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<so101_direct_driver::FollowJointTrajectoryServerNode>());
  } catch (const std::exception & error) {
    std::cerr << "FollowJointTrajectory server 启动失败：" << error.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
