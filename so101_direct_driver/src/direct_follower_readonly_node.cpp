#include <fcntl.h>
#include <cerrno>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace so101_direct_driver
{

class FeetechReadonlyBus
{
public:
  FeetechReadonlyBus(const std::string & port, int baudrate, double timeout_seconds)
  : timeout_seconds_(timeout_seconds)
  {
    fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd_ < 0) {
      throw std::runtime_error("打开串口失败：" + std::string(std::strerror(errno)));
    }

    try {
      configure_port(baudrate);
      flush();
    } catch (...) {
      close();
      throw;
    }
  }

  ~FeetechReadonlyBus()
  {
    close();
  }

  FeetechReadonlyBus(const FeetechReadonlyBus &) = delete;
  FeetechReadonlyBus & operator=(const FeetechReadonlyBus &) = delete;

  bool ping(uint8_t motor_id)
  {
    send_packet(motor_id, instruction_ping_, {});
    auto packet = read_status_packet(motor_id);
    return packet.first;
  }

  uint16_t read_u16(uint8_t motor_id, uint8_t address)
  {
    send_packet(motor_id, instruction_read_, {address, 2});
    auto packet = read_status_packet(motor_id);
    if (!packet.first) {
      throw std::runtime_error("读取 " + std::to_string(motor_id) + " 号舵机超时。");
    }

    const auto & payload = packet.second;
    if (payload.empty()) {
      throw std::runtime_error("读取 " + std::to_string(motor_id) + " 号舵机失败，返回为空。");
    }

    const uint8_t error = payload[0];
    if (error != 0) {
      throw std::runtime_error(
        "读取 " + std::to_string(motor_id) + " 号舵机失败，错误码：" + std::to_string(error) + "。");
    }

    if (payload.size() < 3) {
      throw std::runtime_error(
        "读取 " + std::to_string(motor_id) + " 号舵机失败，返回数据长度不足。");
    }

    return static_cast<uint16_t>(payload[1]) |
           (static_cast<uint16_t>(payload[2]) << 8);
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
    if (cfsetispeed(&tty, speed) != 0 || cfsetospeed(&tty, speed) != 0) {
      throw std::runtime_error("设置串口波特率失败：" + std::string(std::strerror(errno)));
    }

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
        throw std::runtime_error("当前 C++ 节点暂不支持该波特率：" + std::to_string(baudrate));
    }
  }

  void flush()
  {
    tcflush(fd_, TCIOFLUSH);
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
      if (!read_byte(second, deadline)) {
        return {false, {}};
      }
      if (second != 0xff) {
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
      const uint8_t expected_checksum = static_cast<uint8_t>((~sum) & 0xff);
      if (checksum != expected_checksum) {
        continue;
      }

      if (motor_id != expected_id) {
        continue;
      }

      payload.pop_back();
      return {true, payload};
    }

    return {false, {}};
  }

  bool read_byte(
    uint8_t & value,
    const std::chrono::steady_clock::time_point & deadline)
  {
    while (std::chrono::steady_clock::now() < deadline) {
      fd_set read_fds;
      FD_ZERO(&read_fds);
      FD_SET(fd_, &read_fds);

      const auto remaining = deadline - std::chrono::steady_clock::now();
      const auto remaining_us =
        std::chrono::duration_cast<std::chrono::microseconds>(remaining).count();
      timeval timeout {};
      timeout.tv_sec = static_cast<time_t>(remaining_us / 1000000);
      timeout.tv_usec = static_cast<suseconds_t>(remaining_us % 1000000);

      const int result = select(fd_ + 1, &read_fds, nullptr, nullptr, &timeout);
      if (result < 0) {
        if (errno == EINTR) {
          continue;
        }
        throw std::runtime_error("读取串口数据失败：" + std::string(std::strerror(errno)));
      }
      if (result == 0) {
        return false;
      }

      const ssize_t count = ::read(fd_, &value, 1);
      if (count == 1) {
        return true;
      }
      if (count < 0 && errno != EAGAIN && errno != EINTR) {
        throw std::runtime_error("读取串口数据失败：" + std::string(std::strerror(errno)));
      }
    }

    return false;
  }

  int fd_ {-1};
  double timeout_seconds_ {0.2};
  static constexpr uint8_t instruction_ping_ = 0x01;
  static constexpr uint8_t instruction_read_ = 0x02;
};

class DirectFollowerReadonlyNode : public rclcpp::Node
{
public:
  DirectFollowerReadonlyNode()
  : Node("so101_direct_follower_readonly")
  {
    declare_parameters();
    read_parameters();
    validate_config();

    RCLCPP_INFO(
      get_logger(), "准备打开从臂串口：%s，波特率：%d。", port_.c_str(), baudrate_);

    bus_ = std::make_unique<FeetechReadonlyBus>(port_, baudrate_, response_timeout_);

    RCLCPP_INFO(get_logger(), "串口已打开，开始检测 1-6 号舵机。");
    ping_motors();

    joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    raw_joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states_raw", 10);

    const auto timer_period =
      std::chrono::duration<double>(1.0 / read_rate_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
      std::bind(&DirectFollowerReadonlyNode::publish_positions, this));

    RCLCPP_INFO(get_logger(), "只读连接测试节点已启动。该节点不会发送运动目标。");
  }

  ~DirectFollowerReadonlyNode() override
  {
    if (bus_) {
      bus_->close();
      RCLCPP_INFO(get_logger(), "串口已关闭。");
    }
  }

private:
  void declare_parameters()
  {
    declare_parameter<std::string>("port", "/dev/ttyACM0");
    declare_parameter<int>("baudrate", 1000000);
    declare_parameter<double>("read_rate", 5.0);
    declare_parameter<int>("present_position_address", 56);
    declare_parameter<double>("position_ticks_per_turn", 4096.0);
    declare_parameter<double>("response_timeout", 0.2);
    declare_parameter<std::vector<std::string>>(
      "joint_names",
      {"shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"});
    declare_parameter<std::vector<int64_t>>("motor_ids", {1, 2, 3, 4, 5, 6});
    declare_parameter<std::vector<int64_t>>("homing_offsets", {0, 0, 0, 0, 0, 0});
    declare_parameter<std::vector<int64_t>>("range_mins", {0, 0, 0, 0, 0, 0});
    declare_parameter<std::vector<int64_t>>("range_maxs", {4095, 4095, 4095, 4095, 4095, 4095});
    declare_parameter<std::vector<int64_t>>("signs", {1, 1, 1, 1, 1, 1});
    declare_parameter<std::vector<int64_t>>("zero_positions", {2048, 2048, 2048, 2048, 2048, 2048});
  }

  void read_parameters()
  {
    port_ = get_parameter("port").as_string();
    baudrate_ = static_cast<int>(get_parameter("baudrate").as_int());
    read_rate_ = get_parameter("read_rate").as_double();
    position_address_ = static_cast<uint8_t>(get_parameter("present_position_address").as_int());
    ticks_per_turn_ = get_parameter("position_ticks_per_turn").as_double();
    response_timeout_ = get_parameter("response_timeout").as_double();
    joint_names_ = get_parameter("joint_names").as_string_array();
    motor_ids_ = to_int_vector(get_parameter("motor_ids").as_integer_array());
    homing_offsets_ = to_int_vector(get_parameter("homing_offsets").as_integer_array());
    range_mins_ = to_int_vector(get_parameter("range_mins").as_integer_array());
    range_maxs_ = to_int_vector(get_parameter("range_maxs").as_integer_array());
    signs_ = to_int_vector(get_parameter("signs").as_integer_array());
    zero_positions_ = to_int_vector(get_parameter("zero_positions").as_integer_array());
  }

  std::vector<int> to_int_vector(const std::vector<int64_t> & values) const
  {
    std::vector<int> result;
    result.reserve(values.size());
    for (const auto value : values) {
      result.push_back(static_cast<int>(value));
    }
    return result;
  }

  void validate_config()
  {
    const auto expected_size = joint_names_.size();
    validate_size("motor_ids", motor_ids_, expected_size);
    validate_size("homing_offsets", homing_offsets_, expected_size);
    validate_size("range_mins", range_mins_, expected_size);
    validate_size("range_maxs", range_maxs_, expected_size);
    validate_size("signs", signs_, expected_size);
    validate_size("zero_positions", zero_positions_, expected_size);

    if (read_rate_ <= 0.0) {
      throw std::runtime_error("read_rate 必须大于 0。");
    }
    if (ticks_per_turn_ <= 0.0) {
      throw std::runtime_error("position_ticks_per_turn 必须大于 0。");
    }
  }

  void validate_size(const std::string & name, const std::vector<int> & values, size_t expected_size)
  {
    if (values.size() != expected_size) {
      throw std::runtime_error(
        "参数 " + name + " 的数量是 " + std::to_string(values.size()) +
        "，但关节数量是 " + std::to_string(expected_size) + "。");
    }
  }

  void ping_motors()
  {
    std::vector<std::string> missing;
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      const auto motor_id = static_cast<uint8_t>(motor_ids_[i]);
      if (bus_->ping(motor_id)) {
        RCLCPP_INFO(
          get_logger(), "%s：检测到 %d 号舵机。", joint_names_[i].c_str(), motor_ids_[i]);
      } else {
        missing.push_back(joint_names_[i] + "(ID " + std::to_string(motor_ids_[i]) + ")");
      }
    }

    if (!missing.empty()) {
      std::string message = "以下舵机没有响应：";
      for (size_t i = 0; i < missing.size(); ++i) {
        if (i > 0) {
          message += "，";
        }
        message += missing[i];
      }
      message += "。";
      throw std::runtime_error(message);
    }

    RCLCPP_INFO(get_logger(), "所有舵机均已响应。");
  }

  void publish_positions()
  {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = get_clock()->now();
    msg.name = joint_names_;
    msg.position.reserve(joint_names_.size());
    msg.velocity.assign(joint_names_.size(), 0.0);

    std::vector<int> raw_positions;
    raw_positions.reserve(joint_names_.size());

    for (size_t i = 0; i < joint_names_.size(); ++i) {
      int raw_position = 0;
      try {
        raw_position = static_cast<int>(
          bus_->read_u16(static_cast<uint8_t>(motor_ids_[i]), position_address_));
      } catch (const std::exception & error) {
        RCLCPP_ERROR(
          get_logger(), "%s：读取位置失败：%s", joint_names_[i].c_str(), error.what());
        return;
      }

      raw_positions.push_back(raw_position);
      if (raw_position < range_mins_[i] || raw_position > range_maxs_[i]) {
        RCLCPP_WARN(
          get_logger(), "%s：当前位置 %d 超出标定范围 [%d, %d]。",
          joint_names_[i].c_str(), raw_position, range_mins_[i], range_maxs_[i]);
      }

      msg.position.push_back(raw_position_to_radians(raw_position, zero_positions_[i], signs_[i]));
    }

    if (raw_log_count_ < 5) {
      RCLCPP_INFO(get_logger(), "当前舵机原始位置：%s", format_positions(raw_positions).c_str());
      ++raw_log_count_;
    }

    joint_state_pub_->publish(msg);
    raw_joint_state_pub_->publish(msg);
  }

  double raw_position_to_radians(int raw_position, int zero_position, int sign) const
  {
    return static_cast<double>(sign) *
           ((static_cast<double>(raw_position - zero_position) / ticks_per_turn_) * 2.0 * pi_);
  }

  std::string format_positions(const std::vector<int> & values) const
  {
    std::string result = "[";
    for (size_t i = 0; i < values.size(); ++i) {
      if (i > 0) {
        result += ", ";
      }
      result += std::to_string(values[i]);
    }
    result += "]";
    return result;
  }

  std::string port_;
  int baudrate_ {1000000};
  double read_rate_ {5.0};
  uint8_t position_address_ {56};
  double ticks_per_turn_ {4096.0};
  double response_timeout_ {0.2};
  std::vector<std::string> joint_names_;
  std::vector<int> motor_ids_;
  std::vector<int> homing_offsets_;
  std::vector<int> range_mins_;
  std::vector<int> range_maxs_;
  std::vector<int> signs_;
  std::vector<int> zero_positions_;
  int raw_log_count_ {0};

  std::unique_ptr<FeetechReadonlyBus> bus_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr raw_joint_state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  static constexpr double pi_ = 3.14159265358979323846;
};

}  // namespace so101_direct_driver

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<so101_direct_driver::DirectFollowerReadonlyNode>();
    rclcpp::spin(node);
  } catch (const std::exception & error) {
    std::cerr << "只读连接测试失败：" << error.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
