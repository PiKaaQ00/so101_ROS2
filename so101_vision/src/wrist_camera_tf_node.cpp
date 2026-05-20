#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"

namespace so101_vision
{

class WristCameraTfNode : public rclcpp::Node
{
public:
  WristCameraTfNode()
  : Node("wrist_camera_tf_node")
  {
    declare_parameter<std::string>("parent_frame", "follower/gripper_link");
    declare_parameter<std::string>("camera_frame", "camera_link");
    declare_parameter<std::string>("optical_frame", "camera_optical_frame");
    declare_parameter<double>("x", -0.005);
    declare_parameter<double>("y", 0.032);
    declare_parameter<double>("z", -0.020);
    declare_parameter<double>("roll", 0.0);
    declare_parameter<double>("pitch", 0.0);
    declare_parameter<double>("yaw", 1.5708);

    broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    publish_transform();
  }

private:
  void publish_transform()
  {
    const auto parent_frame = get_parameter("parent_frame").as_string();
    const auto camera_frame = get_parameter("camera_frame").as_string();
    const auto optical_frame = get_parameter("optical_frame").as_string();
    const auto x = get_parameter("x").as_double();
    const auto y = get_parameter("y").as_double();
    const auto z = get_parameter("z").as_double();
    const auto roll = get_parameter("roll").as_double();
    const auto pitch = get_parameter("pitch").as_double();
    const auto yaw = get_parameter("yaw").as_double();

    tf2::Quaternion quaternion;
    quaternion.setRPY(roll, pitch, yaw);
    quaternion.normalize();

    geometry_msgs::msg::TransformStamped camera_transform;
    camera_transform.header.stamp = get_clock()->now();
    camera_transform.header.frame_id = parent_frame;
    camera_transform.child_frame_id = camera_frame;
    camera_transform.transform.translation.x = x;
    camera_transform.transform.translation.y = y;
    camera_transform.transform.translation.z = z;
    camera_transform.transform.rotation.x = quaternion.x();
    camera_transform.transform.rotation.y = quaternion.y();
    camera_transform.transform.rotation.z = quaternion.z();
    camera_transform.transform.rotation.w = quaternion.w();

    tf2::Quaternion optical_quaternion;
    optical_quaternion.setRPY(-1.57079632679, 0.0, -1.57079632679);
    optical_quaternion.normalize();

    geometry_msgs::msg::TransformStamped optical_transform;
    optical_transform.header.stamp = camera_transform.header.stamp;
    optical_transform.header.frame_id = camera_frame;
    optical_transform.child_frame_id = optical_frame;
    optical_transform.transform.translation.x = 0.0;
    optical_transform.transform.translation.y = 0.0;
    optical_transform.transform.translation.z = 0.0;
    optical_transform.transform.rotation.x = optical_quaternion.x();
    optical_transform.transform.rotation.y = optical_quaternion.y();
    optical_transform.transform.rotation.z = optical_quaternion.z();
    optical_transform.transform.rotation.w = optical_quaternion.w();

    broadcaster_->sendTransform(std::vector<geometry_msgs::msg::TransformStamped>{
      camera_transform,
      optical_transform});

    RCLCPP_INFO(
      get_logger(),
      "Published wrist camera TF: %s -> %s, xyz=(%.3f, %.3f, %.3f), rpy=(%.3f, %.3f, %.3f)",
      parent_frame.c_str(),
      camera_frame.c_str(),
      x,
      y,
      z,
      roll,
      pitch,
      yaw);
    RCLCPP_INFO(
      get_logger(),
      "Published camera optical TF: %s -> %s",
      camera_frame.c_str(),
      optical_frame.c_str());
  }

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
};

}  // namespace so101_vision

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<so101_vision::WristCameraTfNode>());
  rclcpp::shutdown();
  return 0;
}
