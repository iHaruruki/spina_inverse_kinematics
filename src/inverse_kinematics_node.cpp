#include <memory>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace Eigen;

constexpr double DEG2RAD = M_PI / 180.0;
constexpr double MAX_TILT_RAD = 30.0 * DEG2RAD;

class IKNode : public rclcpp::Node {
public:
  IKNode(): Node("inverse_kinematics_node") {
    joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "target_pose", 10,
      std::bind(&IKNode::poseCallback, this, std::placeholders::_1)
    );
    joint_state_.name.resize(6);
    for (int i = 0; i < 6; ++i) {
      joint_state_.name[i] = "joint" + std::to_string(i+1);
    }
    joint_state_.position.resize(6, 0.0);
  }

private:
  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    // 同次変換行列を構成
    Matrix4d target = Matrix4d::Identity();
    auto &p = msg->pose.position;
    target(0,3) = p.x;
    target(1,3) = p.y;
    target(2,3) = p.z;
    // IK計算
    double x = target(0,3), y = target(1,3), z = target(2,3);
    double base = std::atan2(y,x);
    double r = std::hypot(x,y);
    double overall_tilt = std::atan2(z,r);
    double per = overall_tilt / 5.0;
    if (std::abs(per) > MAX_TILT_RAD) per = (per>0?MAX_TILT_RAD:-MAX_TILT_RAD);

    joint_state_.header.stamp = this->now();
    joint_state_.position[0] = base;
    for (int i=1; i<6; ++i) joint_state_.position[i] = per;
    joint_pub_->publish(joint_state_);
  }

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  sensor_msgs::msg::JointState joint_state_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IKNode>());
  rclcpp::shutdown();
  return 0;
}
