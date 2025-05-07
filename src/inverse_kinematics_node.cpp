#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class SpineIKNode : public rclcpp::Node
{
public:
  SpineIKNode()
  : Node("spine_ik_node"), num_modules_(6)
  {
    // Declare and get parameters
    this->declare_parameter<double>("link_length", 0.15);
    link_length_ = this->get_parameter("link_length").as_double();

    // The total link length is used for computing the expected end-effector pose.
    total_length_ = link_length_ * num_modules_;

    // Create Subscription to "endpoint" topic (the end-effector coordinates)
    subscription_ = this->create_subscription<geometry_msgs::msg::Point>(
      "endpoint", 10, std::bind(&SpineIKNode::endpointCallback, this, _1));

    // Create Publisher for joint_states
    joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
  }

private:
  void endpointCallback(const geometry_msgs::msg::Point::SharedPtr msg)
  {
    // When computing individual joint angles per module,
    // each module is assumed to contribute equally. Thus, for each module:
    //   module_pitch = asin( x / (num_modules * link_length) )
    //   module_roll  = asin( y / (num_modules * link_length) )
    // The overall computed z is then:
    //   computed_z = num_modules * link_length * cos(module_pitch) * cos(module_roll)

    // Validate x/y values so that the argument of asin remains in [-1, 1]
    if (std::abs(msg->x) > total_length_ || std::abs(msg->y) > total_length_) {
      RCLCPP_WARN(this->get_logger(),
        "x or y value (x: %.3f, y: %.3f) exceeds total link length (%.3f m).",
        msg->x, msg->y, total_length_);
      return;
    }

    double module_pitch = 0.0;
    double module_roll = 0.0;
    try {
      module_pitch = std::asin(msg->x / (num_modules_ * link_length_));
      module_roll  = std::asin(msg->y / (num_modules_ * link_length_));
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Inverse kinematics calculation failed: %s", e.what());
      return;
    }

    double computed_z = num_modules_ * link_length_ * std::cos(module_pitch) * std::cos(module_roll);
    if (std::abs(computed_z - msg->z) > 0.01) {
      RCLCPP_WARN(this->get_logger(),
        "Computed z (%.3f m) does not match provided z (%.3f m).", computed_z, msg->z);
    }

    // Publish joint states for each module.
    // Each module has two joints: pitch and roll.
    auto joint_state_msg = sensor_msgs::msg::JointState();
    joint_state_msg.header.stamp = this->now();

    for (int i = 1; i <= num_modules_; ++i) {
      joint_state_msg.name.push_back("pitch" + std::to_string(i));
      joint_state_msg.position.push_back(module_pitch);
      joint_state_msg.name.push_back("roll" + std::to_string(i));
      joint_state_msg.position.push_back(module_roll);
    }

    joint_pub_->publish(joint_state_msg);
    RCLCPP_INFO(this->get_logger(),
      "Published joint angles for each module: pitch = %.3f, roll = %.3f", module_pitch, module_roll);
  }

  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  double link_length_;
  double total_length_;
  const int num_modules_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SpineIKNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
