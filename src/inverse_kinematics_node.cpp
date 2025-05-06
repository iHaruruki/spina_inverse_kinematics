#include <memory>
#include <vector>
#include <cmath>
#include <algorithm>
#include <Eigen/Dense>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace Eigen;
using std::vector;

constexpr double DEG2RAD    = M_PI / 180.0;
constexpr double MAX_TILT   = 30.0 * DEG2RAD;

class IKNode : public rclcpp::Node {
public:
  IKNode(): Node("inverse_kinematics_node") {
    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "target_pose", 10,
      std::bind(&IKNode::poseCallback, this, std::placeholders::_1)
    );

    // ジョイント名 (yaw, then pitch1,roll1, … pitch6,roll6)
    joint_state_.name.push_back("yaw");
    for(int i=1; i<=6; ++i) {
      joint_state_.name.push_back("pitch"+std::to_string(i));
      joint_state_.name.push_back("roll"+ std::to_string(i));
    }
    joint_state_.position.assign(joint_state_.name.size(), 0.0);
  }

private:
  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    // 目標位置
    double x = msg->pose.position.x;
    double y = msg->pose.position.y;
    double z = msg->pose.position.z;

    // 基底 yaw
    double yaw = std::atan2(y, x);

    // 目標姿勢から roll, pitch を抽出
    Quaterniond q(
      msg->pose.orientation.w,
      msg->pose.orientation.x,
      msg->pose.orientation.y,
      msg->pose.orientation.z
    );
    Matrix3d R = q.toRotationMatrix();
    // ZYX 順で Euler: [yaw, pitch, roll]
    Vector3d e = R.eulerAngles(2,1,0);
    double target_pitch = e[1];
    double target_roll  = e[2];

    // モジュール毎に均等分配
    double per_pitch = target_pitch / 6.0;
    double per_roll  = target_roll  / 6.0;
    per_pitch = std::min(std::max(per_pitch, -MAX_TILT), MAX_TILT);
    per_roll  = std::min(std::max(per_roll,  -MAX_TILT), MAX_TILT);

    // joint_state を設定
    joint_state_.header.stamp = now();
    size_t idx = 0;
    joint_state_.position[idx++] = yaw;
    for(int i=0; i<6; ++i) {
      joint_state_.position[idx++] = per_pitch;
      joint_state_.position[idx++] = per_roll;
    }
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
