#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <Eigen/Dense>
#include <cmath>

using namespace std::chrono_literals;

// Helper functions to build homogeneous transformation matrices.
Eigen::Matrix4d translate(double x, double y, double z) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T(0, 3) = x;
  T(1, 3) = y;
  T(2, 3) = z;
  return T;
}

Eigen::Matrix4d rotX(double angle_rad) {
  Eigen::Matrix4d R = Eigen::Matrix4d::Identity();
  double c = std::cos(angle_rad);
  double s = std::sin(angle_rad);
  R(1,1) = c;  R(1,2) = -s;
  R(2,1) = s;  R(2,2) = c;
  return R;
}

Eigen::Matrix4d rotY(double angle_rad) {
  Eigen::Matrix4d R = Eigen::Matrix4d::Identity();
  double c = std::cos(angle_rad);
  double s = std::sin(angle_rad);
  R(0,0) = c;  R(0,2) = s;
  R(2,0) = -s; R(2,2) = c;
  return R;
}

// Compute the transformation for one module
// The transform follows the sequence:
//    fixed translation (0,0,0.075) -> roll rotation about x -> pitch rotation about y -> fixed translation (0,0,0.075)
Eigen::Matrix4d computeModuleTransform(double roll, double pitch) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T = translate(0, 0, 0.075) * T;
  T = rotX(roll) * T;
  T = rotY(pitch) * T;
  T = translate(0, 0, 0.075) * T;
  return T;
}

// Utility function: get the joint angle by name from a joint state message, or return 0 if not found.
double getJointAngle(const sensor_msgs::msg::JointState::SharedPtr msg, const std::string & joint_name) {
  for (size_t i = 0; i < msg->name.size(); ++i) {
    if (msg->name[i] == joint_name) {
      return msg->position[i];
    }
  }
  return 0.0;
}

class ForwardKinematicsNode : public rclcpp::Node
{
public:
  ForwardKinematicsNode()
  : Node("forward_kinematics_node")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10,
      std::bind(&ForwardKinematicsNode::jointStateCallback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Forward kinematics node has been started.");
  }

private:
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    // Assume the robot has 6 modules and for each module, the joints are:
    // "module{i}_gimbal_roll" and "module{i}_gimbal_pitch" for modules 1 to 6
    const int num_modules = 6;
    Eigen::Matrix4d T_total = Eigen::Matrix4d::Identity();

    // Process each module in order
    for (int i = 1; i <= num_modules; ++i) {
      std::string roll_joint = "module" + std::to_string(i) + "_gimbal_roll";
      std::string pitch_joint = "module" + std::to_string(i) + "_gimbal_pitch";

      double roll_angle  = getJointAngle(msg, roll_joint);
      double pitch_angle = getJointAngle(msg, pitch_joint);

      // Compute transformation for this module
      Eigen::Matrix4d T_module = computeModuleTransform(roll_angle, pitch_angle);
      T_total = T_total * T_module;
    }

    // Extract the position of the end effector (hand) from T_total (translation part)
    Eigen::Vector3d position = T_total.block<3, 1>(0, 3);
    RCLCPP_INFO(this->get_logger(), "End-effector position: x: %.3f, y: %.3f, z: %.3f",
                position.x(), position.y(), position.z());
  }

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ForwardKinematicsNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
