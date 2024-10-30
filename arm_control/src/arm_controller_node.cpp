#include <functional>
#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using std::placeholders::_1;

class ArmControllerNode : public rclcpp::Node
{
public:
  ArmControllerNode()
  : Node("arm_controller_node")
  {
    joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10, std::bind(&ArmControllerNode::jointState_callback, this, _1));

    position_command_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/position_controller/commands", 10);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&ArmControllerNode::timer_callback, this));
  }

private:
  void jointState_callback(const sensor_msgs::msg::JointState::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Current joint positions:");
    for (size_t i = 0; i < msg->position.size(); ++i) {
    RCLCPP_INFO(this->get_logger(), "Joint %ld: %f", i, msg->position[i]);
    }
  }

  void timer_callback()
  {
    auto message = std_msgs::msg::Float64MultiArray();
    
    message.data = {1.0, 0.5, 0.0, 0.1};

    RCLCPP_INFO(this->get_logger(), "Published position command: [%f, %f, %f, %f]", message.data[0], message.data[1], message.data[2], message.data[3]); 

    position_command_publisher_->publish(message);

  }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr position_command_publisher_;


};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArmControllerNode>());
  rclcpp::shutdown();
  return 0;
}