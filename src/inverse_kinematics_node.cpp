#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <vector>
#include <string>

class IKJointStatePublisher : public rclcpp::Node
{
public:
  IKJointStatePublisher() : Node("ik_joint_state_publisher")
  {
    // Subscribe to endpoint coordinates (assumed on topic "endpoint")
    endpoint_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
        "endpoint", 10,
        std::bind(&IKJointStatePublisher::endpointCallback, this, std::placeholders::_1));

    // Publisher for JointState messages
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    // Populate joint names (for 6 modules with roll and pitch each)
    // Modify names as needed to match URDF configuration.
    joint_names_ = { "module1_roll", "module1_pitch",
                     "module2_roll", "module2_pitch",
                     "module3_roll", "module3_pitch",
                     "module4_roll", "module4_pitch",
                     "module5_roll", "module5_pitch",
                     "module6_roll", "module6_pitch" };
  }

private:
  void endpointCallback(const geometry_msgs::msg::Point::SharedPtr msg)
  {
    double x = msg->x;
    double y = msg->y;
    double z = msg->z;
    RCLCPP_INFO(this->get_logger(), "Received endpoint: [%.2f, %.2f, %.2f]", x, y, z);

    // Compute inverse kinematics to determine joint angles for modules 1-6.
    // This function returns a vector of 12 angles (dummy implementation).
    std::vector<double> joint_angles = computeIK(x, y, z);

    // Prepare and publish a JointState message.
    sensor_msgs::msg::JointState joint_state_msg;
    joint_state_msg.header.stamp = this->get_clock()->now();
    joint_state_msg.name = joint_names_;
    joint_state_msg.position = joint_angles;

    joint_state_pub_->publish(joint_state_msg);

    // Log published joint states.
    std::string angle_str;
    for (size_t i = 0; i < joint_angles.size(); i++) {
      angle_str += joint_names_[i] + ": " + std::to_string(joint_angles[i]) + " [rad]   ";
    }
    RCLCPP_INFO(this->get_logger(), "Published JointState: %s", angle_str.c_str());
  }

  // Dummy IK computation returning 12 joint angles.
  // Replace this function with actual inverse kinematics computations.
  std::vector<double> computeIK(double x, double y, double z)
  {
    std::vector<double> angles(12, 0.0);
    // For demonstration, we'll compute dummy angles based on endpoint data.
    // For each module, roll gets a fraction of x and pitch a fraction of y.
    for (int i = 0; i < 6; ++i)
    {
      angles[2 * i]     = (x / 10.0) + (i * 0.05);  // roll angle in radians
      angles[2 * i + 1] = (y / 10.0) + (i * 0.03);  // pitch angle in radians
    }
    return angles;
  }

  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr endpoint_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  std::vector<std::string> joint_names_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<IKJointStatePublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
