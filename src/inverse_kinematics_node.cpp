#include <memory>
#include <vector>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <Eigen/Dense>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace Eigen;
using namespace std::chrono_literals;

constexpr double DEG2RAD    = M_PI / 180.0;
constexpr double MAX_TILT   = 30.0 * DEG2RAD;

class IKNode : public rclcpp::Node {
public:
  IKNode()
  : Node("inverse_kinematics_node")
  {
    // joint_states を transient_local QoS で作成し、最後のメッセージを保持
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
    joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", qos);

    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "target_pose", 10,
      std::bind(&IKNode::poseCallback, this, std::placeholders::_1)
    );

    // 定期的に joint_states を再送信するタイマー (10Hz)
    publish_timer_ = this->create_wall_timer(
      100ms,
      std::bind(&IKNode::onTimer, this)
    );

    // joint_state の名前をセット (yaw, then pitch1,roll1, … pitch6,roll6)
    joint_state_.name.push_back("yaw");
    for (int i = 1; i <= 6; ++i) {
      joint_state_.name.push_back("pitch" + std::to_string(i));
      joint_state_.name.push_back("roll"  + std::to_string(i));
    }
    joint_state_.position.assign(joint_state_.name.size(), 0.0);
  }

private:
  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    // 目標位置
    double x = msg->pose.position.x;
    double y = msg->pose.position.y;
    double z = msg->pose.position.z;

    // 基底 yaw
    double yaw = std::atan2(y, x);

    // 目標姿勢から ZYX の Euler を取得 (yaw, pitch, roll)
    Quaterniond q(
      msg->pose.orientation.w,
      msg->pose.orientation.x,
      msg->pose.orientation.y,
      msg->pose.orientation.z
    );
    Matrix3d R = q.toRotationMatrix();
    auto e = R.eulerAngles(2, 1, 0);
    double target_pitch = e[1];
    double target_roll  = e[2];

    // 各モジュールに均等分配しつつクランプ
    double per_pitch = std::clamp(target_pitch / 6.0, -MAX_TILT, MAX_TILT);
    double per_roll  = std::clamp(target_roll  / 6.0, -MAX_TILT, MAX_TILT);

    // joint_state を更新
    joint_state_.header.stamp = this->now();
    size_t idx = 0;
    joint_state_.position[idx++] = yaw;
    for (int i = 0; i < 6; ++i) {
      joint_state_.position[idx++] = per_pitch;
      joint_state_.position[idx++] = per_roll;
    }

    // 即時にパブリッシュ
    joint_pub_->publish(joint_state_);
  }

  void onTimer()
  {
    // 最後に計算した joint_state_ を再送信
    joint_state_.header.stamp = this->now();
    joint_pub_->publish(joint_state_);
  }

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  sensor_msgs::msg::JointState joint_state_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IKNode>());
  rclcpp::shutdown();
  return 0;
}
