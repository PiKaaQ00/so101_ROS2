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

  FeetechBus(const FeetechBus &) = delete;
  FeetechBus & operator=(const FeetechBus &) = delete;

  bool ping(uint8_t motor_id)
  {
    send_packet(motor_id, instruction_ping_, {});
    return read_status_packet(motor_id).first;
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
    if (payload[0] != 0) {
      throw std::runtime_error(
        "读取 " + std::to_string(motor_id) + " 号舵机失败，错误码：" +
        std::to_string(payload[0]) + "。");
    }
    if (payload.size() < 3) {
      throw std::runtime_error("读取 " + std::to_string(motor_id) + " 号舵机失败，返回数据长度不足。");
    }

    return static_cast<uint16_t>(payload[1]) |
           static_cast<uint16_t>(payload[2] << 8);
  }

  void write_u16(uint8_t motor_id, uint8_t address, uint16_t value)
  {
    const std::vector<uint8_t> params = {
      address,
      static_cast<uint8_t>(value & 0xff),
      static_cast<uint8_t>((value >> 8) & 0xff),
    };
    send_packet(motor_id, instruction_write_, params);
    auto packet = read_status_packet(motor_id);
    if (!packet.first) {
      throw std::runtime_error("写入 " + std::to_string(motor_id) + " 号舵机超时。");
    }
    if (!packet.second.empty() && packet.second[0] != 0) {
      throw std::runtime_error(
        "写入 " + std::to_string(motor_id) + " 号舵机失败，错误码：" +
        std::to_string(packet.second[0]) + "。");
    }
  }

  void write_u8(uint8_t motor_id, uint8_t address, uint8_t value)
  {
    send_packet(motor_id, instruction_write_, {address, value});
    auto packet = read_status_packet(motor_id);
    if (!packet.first) {
      throw std::runtime_error("写入 " + std::to_string(motor_id) + " 号舵机超时。");
    }
    if (!packet.second.empty() && packet.second[0] != 0) {
      throw std::runtime_error(
        "写入 " + std::to_string(motor_id) + " 号舵机失败，错误码：" +
        std::to_string(packet.second[0]) + "。");
    }
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
      if (checksum != static_cast<uint8_t>((~sum) & 0xff)) {
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

  bool read_byte(uint8_t & value, const std::chrono::steady_clock::time_point & deadline)
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
  static constexpr uint8_t instruction_write_ = 0x03;
};

class SingleJointWriteTestNode : public rclcpp::Node
{
public:
  SingleJointWriteTestNode()
  : Node("so101_single_joint_write_test")
  {
    declare_parameters();
    read_parameters();
    validate_config();

    RCLCPP_INFO(get_logger(), "准备打开从臂串口：%s，波特率：%d。", port_.c_str(), baudrate_);
    bus_ = std::make_unique<FeetechBus>(port_, baudrate_, response_timeout_);
    RCLCPP_INFO(get_logger(), "串口已打开，开始检测 1-6 号舵机。");
    ping_motors();

    joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    raw_joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states_raw", 10);

    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / read_rate_)),
      std::bind(&SingleJointWriteTestNode::timer_callback, this));

    RCLCPP_WARN(get_logger(), "单关节写入测试节点已启动。默认只预演，enable_write=true 才会写入一次。");
  }

  ~SingleJointWriteTestNode() override
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
    declare_parameter<int>("goal_position_address", 42);
    declare_parameter<int>("torque_enable_address", 40);
    declare_parameter<bool>("enable_torque_before_write", true);
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
    declare_parameter<std::string>("joint", "gripper");
    declare_parameter<double>("delta_rad", 0.01);
    declare_parameter<double>("max_abs_delta_rad", 0.05);
    declare_parameter<bool>("enable_write", false);
  }

  void read_parameters()
  {
    port_ = get_parameter("port").as_string();
    baudrate_ = static_cast<int>(get_parameter("baudrate").as_int());
    read_rate_ = get_parameter("read_rate").as_double();
    present_position_address_ = static_cast<uint8_t>(get_parameter("present_position_address").as_int());
    goal_position_address_ = static_cast<uint8_t>(get_parameter("goal_position_address").as_int());
    torque_enable_address_ = static_cast<uint8_t>(get_parameter("torque_enable_address").as_int());
    enable_torque_before_write_ = get_parameter("enable_torque_before_write").as_bool();
    ticks_per_turn_ = get_parameter("position_ticks_per_turn").as_double();
    response_timeout_ = get_parameter("response_timeout").as_double();
    joint_names_ = get_parameter("joint_names").as_string_array();
    motor_ids_ = to_int_vector(get_parameter("motor_ids").as_integer_array());
    homing_offsets_ = to_int_vector(get_parameter("homing_offsets").as_integer_array());
    range_mins_ = to_int_vector(get_parameter("range_mins").as_integer_array());
    range_maxs_ = to_int_vector(get_parameter("range_maxs").as_integer_array());
    signs_ = to_int_vector(get_parameter("signs").as_integer_array());
    zero_positions_ = to_int_vector(get_parameter("zero_positions").as_integer_array());
    target_joint_ = get_parameter("joint").as_string();
    delta_rad_ = get_parameter("delta_rad").as_double();
    max_abs_delta_rad_ = get_parameter("max_abs_delta_rad").as_double();
    enable_write_ = get_parameter("enable_write").as_bool();
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
    const auto size = joint_names_.size();
    validate_size("motor_ids", motor_ids_, size);
    validate_size("homing_offsets", homing_offsets_, size);
    validate_size("range_mins", range_mins_, size);
    validate_size("range_maxs", range_maxs_, size);
    validate_size("signs", signs_, size);
    validate_size("zero_positions", zero_positions_, size);

    target_joint_index_ = -1;
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      if (joint_names_[i] == target_joint_) {
        target_joint_index_ = static_cast<int>(i);
        break;
      }
    }
    if (target_joint_index_ < 0) {
      throw std::runtime_error("找不到目标关节：" + target_joint_);
    }
    if (std::abs(delta_rad_) > max_abs_delta_rad_) {
      throw std::runtime_error(
        "delta_rad 超过安全限制。当前：" + std::to_string(delta_rad_) +
        "，限制：" + std::to_string(max_abs_delta_rad_) + "。");
    }
    if (read_rate_ <= 0.0 || ticks_per_turn_ <= 0.0) {
      throw std::runtime_error("read_rate 和 position_ticks_per_turn 必须大于 0。");
    }
  }

  void validate_size(const std::string & name, const std::vector<int> & values, size_t expected)
  {
    if (values.size() != expected) {
      throw std::runtime_error("参数 " + name + " 数量不匹配。");
    }
  }

  void ping_motors()
  {
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      const auto motor_id = static_cast<uint8_t>(motor_ids_[i]);
      if (!bus_->ping(motor_id)) {
        throw std::runtime_error(joint_names_[i] + "：没有检测到 " + std::to_string(motor_ids_[i]) + " 号舵机。");
      }
      RCLCPP_INFO(get_logger(), "%s：检测到 %d 号舵机。", joint_names_[i].c_str(), motor_ids_[i]);
    }
    RCLCPP_INFO(get_logger(), "所有舵机均已响应。");
  }

  void timer_callback()
  {
    std::vector<int> raw_positions;
    if (!read_all_positions(raw_positions)) {
      return;
    }
    publish_joint_states(raw_positions);

    if (!write_attempted_) {
      write_attempted_ = true;
      run_single_write_check(raw_positions);
    }
  }

  bool read_all_positions(std::vector<int> & raw_positions)
  {
    raw_positions.clear();
    raw_positions.reserve(joint_names_.size());
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      try {
        raw_positions.push_back(
          static_cast<int>(bus_->read_u16(static_cast<uint8_t>(motor_ids_[i]), present_position_address_)));
      } catch (const std::exception & error) {
        RCLCPP_ERROR(get_logger(), "%s：读取位置失败：%s", joint_names_[i].c_str(), error.what());
        return false;
      }
    }
    return true;
  }

  void publish_joint_states(const std::vector<int> & raw_positions)
  {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = get_clock()->now();
    msg.name = joint_names_;
    msg.velocity.assign(joint_names_.size(), 0.0);
    msg.position.reserve(joint_names_.size());

    for (size_t i = 0; i < raw_positions.size(); ++i) {
      msg.position.push_back(raw_position_to_radians(raw_positions[i], zero_positions_[i], signs_[i]));
    }

    joint_state_pub_->publish(msg);
    raw_joint_state_pub_->publish(msg);
  }

  void run_single_write_check(const std::vector<int> & raw_positions)
  {
    const int index = target_joint_index_;
    const int current_raw = raw_positions[index];
    const int delta_ticks = static_cast<int>(
      std::llround(delta_rad_ * ticks_per_turn_ / (2.0 * pi_) / static_cast<double>(signs_[index])));
    const int target_raw = current_raw + delta_ticks;

    RCLCPP_WARN(get_logger(), "准备测试单关节：%s。", target_joint_.c_str());
    RCLCPP_WARN(get_logger(), "当前原始位置：%d，目标原始位置：%d，变化量：%d。", current_raw, target_raw, delta_ticks);
    RCLCPP_WARN(get_logger(), "允许范围：[%d, %d]。", range_mins_[index], range_maxs_[index]);

    if (target_raw < range_mins_[index] || target_raw > range_maxs_[index]) {
      RCLCPP_ERROR(get_logger(), "目标位置超出标定范围，已取消写入。");
      return;
    }

    if (!enable_write_) {
      RCLCPP_WARN(get_logger(), "当前 enable_write=false，只做预演，不会写入舵机。");
      RCLCPP_WARN(get_logger(), "确认安全后，使用 enable_write:=true 再启动。");
      return;
    }

    try {
      if (enable_torque_before_write_) {
        bus_->write_u8(static_cast<uint8_t>(motor_ids_[index]), torque_enable_address_, 1);
        RCLCPP_WARN(get_logger(), "已使能 %s 的舵机扭矩。", target_joint_.c_str());
      }
      bus_->write_u16(
        static_cast<uint8_t>(motor_ids_[index]), goal_position_address_, static_cast<uint16_t>(target_raw));
      RCLCPP_WARN(get_logger(), "已向 %s 发送一次目标位置：%d。", target_joint_.c_str(), target_raw);
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "写入失败：%s", error.what());
    }
  }

  double raw_position_to_radians(int raw_position, int zero_position, int sign) const
  {
    return static_cast<double>(sign) *
           ((static_cast<double>(raw_position - zero_position) / ticks_per_turn_) * 2.0 * pi_);
  }

  std::string port_;
  int baudrate_ {1000000};
  double read_rate_ {5.0};
  uint8_t present_position_address_ {56};
  uint8_t goal_position_address_ {42};
  uint8_t torque_enable_address_ {40};
  bool enable_torque_before_write_ {true};
  double ticks_per_turn_ {4096.0};
  double response_timeout_ {0.2};
  std::vector<std::string> joint_names_;
  std::vector<int> motor_ids_;
  std::vector<int> homing_offsets_;
  std::vector<int> range_mins_;
  std::vector<int> range_maxs_;
  std::vector<int> signs_;
  std::vector<int> zero_positions_;
  std::string target_joint_;
  double delta_rad_ {0.01};
  double max_abs_delta_rad_ {0.05};
  bool enable_write_ {false};
  int target_joint_index_ {-1};
  bool write_attempted_ {false};

  std::unique_ptr<FeetechBus> bus_;
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
    auto node = std::make_shared<so101_direct_driver::SingleJointWriteTestNode>();
    rclcpp::spin(node);
  } catch (const std::exception & error) {
    std::cerr << "单关节写入测试失败：" << error.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
