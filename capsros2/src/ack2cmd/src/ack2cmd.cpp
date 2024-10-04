#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>

class AckermanToCmdVel : public rclcpp::Node
{
public:
  AckermanToCmdVel()
  : Node("ackerman_to_cmdvel")
  {
    ackerman_subscriber_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
      "ackerman_cmd", 10, std::bind(&AckermanToCmdVel::ackermanCmdCallback, this, std::placeholders::_1));

    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    wheelbase_length_ = 0.55;  // 휠베이스 길이 (m)
    track_width_ = 0.6;        // 바퀴 간 거리 (m)
    K_ = track_width_ / wheelbase_length_; // 스케일링 팩터 K 계산
  }

private:
  void ackermanCmdCallback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
  {
    auto cmd_vel_msg = geometry_msgs::msg::Twist();

    cmd_vel_msg.linear.x = msg->drive.speed;
    cmd_vel_msg.angular.z = cmd_vel_msg.linear.x * K_ * std::tan(msg->drive.steering_angle);

    cmd_vel_publisher_->publish(cmd_vel_msg);
    RCLCPP_INFO(this->get_logger(), "Published cmd_vel: linear.x = %.2f, angular.z = %.2f",
                cmd_vel_msg.linear.x, cmd_vel_msg.angular.z);
  }

  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackerman_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  
  float wheelbase_length_;
  float track_width_;
  float K_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AckermanToCmdVel>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
