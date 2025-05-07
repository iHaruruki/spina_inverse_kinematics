#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class TargetPublisher : public rclcpp::Node
{
public:
    TargetPublisher() : Node("target_pose_publisher")
    {
        pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("target_pose", 10);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&TargetPublisher::publish_target, this));
    }

private:
    void publish_target()
    {
        auto msg = geometry_msgs::msg::PoseStamped();
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = "base_link";
        msg.pose.position.x = 1.2;
        msg.pose.position.y = 0.5;
        msg.pose.position.z = 0.8;
        // No rotation (aligned with world axis)
        msg.pose.orientation.x = 0.0;
        msg.pose.orientation.y = 0.0;
        msg.pose.orientation.z = 0.0;
        msg.pose.orientation.w = 1.0;

        pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Publish target: %f, %f, %f",
                     msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TargetPublisher>());
    rclcpp::shutdown();
    return 0;
}
