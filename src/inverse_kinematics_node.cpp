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

constexpr double DEG2RAD  = M_PI / 180.0;
constexpr double MAX_TILT = 30.0 * DEG2RAD;

class IKNode : public rclcpp::Node {
public:
  IKNode(): Node("inverse_kinematics_node") {
    // QoS を transient_local にしてラストメッセージを保持
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", qos);

    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "target_pose", 10,
      std::bind(&IKNode::poseCallback, this, std::placeholders::_1)
    );

    // 定期的に再送信するタイマー（10Hz）
    publish_timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&IKNode::onTimer, this)
    );

    // joint_state の名前をセット
    joint_state_.name.push_back("yaw");
    for(int i=1; i<=6; ++i) {
      joint_state_.name.push_back("pitch"+std::to_string(i));
      joint_state_.name.push_back("roll"+ std::to_string(i));
    }
    joint_state_.position.assign(joint_state_.name.size(), 0.0);
  }

private:
  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    // --- IK 計算 ---
    double x = msg->pose.position.x;
    double y = msg->pose.position.y;
    double z = msg->pose.position.z;

    // yaw
    double yaw = std::atan2(y, x);
    // 目標姿勢から roll/pitch 抽出
    Quaterniond q(
      msg->pose.orientation.w,
      msg->pose.orientation.x,
      msg->pose.orientation.y,
      msg->pose.orientation.z
    );
    Matrix3d R = q.toRotationMatrix();
    Vector3d e = R.eulerAngles(2,1,0);  // ZYX: [yaw, pitch, roll]
    double tgt_pitch = e[1];
    double tgt_roll  = e[2];
    double per_pitch = std::min(std::max(tgt_pitch / 6.0, -MAX_TILT), MAX_TILT);
    double per_roll  = std::min(std::max(tgt_roll  / 6.0, -MAX_TILT), MAX_TILT);

    // joint_state 更新
    joint_state_.header.stamp = now();
    size_t idx = 0;
    joint_state_.position[idx++] = yaw;
    for(int i=0; i<6; ++i) {
      joint_state_.position[idx++] = per_pitch;
      joint_state_.position[idx++] = per_roll;
    }
    // 即時パブリッシュ
    joint_pub_->publish(joint_state_);
  }

  // タイマーコールバック：最後の状態を再パブリッシュ
  void onTimer() {
    joint_state_.header.stamp = now();
    joint_pub_->publish(joint_state_);
  }

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  sensor_msgs::msg::JointState joint_state_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IKNode>());
  rclcpp::shutdown();
  return 0;
}
