#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <cmath>

class CarlControlNode : public rclcpp::Node
{
public:
  CarlControlNode() : Node("carl_control_node")
  {
    declare_parameter("angle_gain", 1.0);
    declare_parameter("speed_gain", 1.0);

    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&CarlControlNode::cmdVelCallback, this, std::placeholders::_1)
    );

    velocity_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/forward_velocity_controller/commands", 10);
    position_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/forward_position_controller/commands", 10);

    RCLCPP_INFO(this->get_logger(), "Nodo avviato. Parametri regolabili: angle_gain, speed_gain");
    sub_tuning_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("/tuning_params", 10, std::bind(&CarlControlNode::tuningCallback, this, std::placeholders::_1));
  }
  void tuningCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() >= 2)
    {
        angle_gain_ = msg->data[0];
        speed_gain_ = msg->data[1];
        RCLCPP_INFO(this->get_logger(), "Parametri aggiornati: angle_gain=%.2f, speed_gain=%.2f", angle_gain_, speed_gain_);
    }
  } 

private:
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_tuning_;
  double angle_gain_ = 1.0;
  double speed_gain_ = 1.0;
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    double linear = msg->linear.x;
    double angular = msg->angular.z;

    double wheel_base = 0.5;
    double track_width = 0.35;

    double angle_gain = angle_gain_;
    double speed_gain = speed_gain_;

    double delta_left = 0.0;
    double delta_right = 0.0;

    if (std::abs(angular) < 1e-5 || std::abs(linear) < 1e-5)
    {
      delta_left = 0.0;
      delta_right = 0.0;
    }
    else
    {
      double turning_radius = linear / angular;
      double abs_radius = std::abs(turning_radius);

      double angle_left = atan2(wheel_base, abs_radius - (track_width / 2.0)) * angle_gain;
      double angle_right = atan2(wheel_base, abs_radius + (track_width / 2.0)) * angle_gain;

      if (turning_radius > 0)
      {
        delta_left = angle_left;
        delta_right = angle_right;
      }
      else
      {
        delta_left = -angle_left;
        delta_right = -angle_right;
      }
    }

    std_msgs::msg::Float64MultiArray vel_msg;
    vel_msg.data = {
      linear * speed_gain,
      linear * speed_gain,
      linear * speed_gain,
      linear * speed_gain
    };
    velocity_pub_->publish(vel_msg);

    std_msgs::msg::Float64MultiArray pos_msg;
    pos_msg.data = { delta_left, delta_right };
    position_pub_->publish(pos_msg);

    RCLCPP_INFO(this->get_logger(), "lin=%.2f ang=%.2f | delta_l=%.3f delta_r=%.3f | angle_gain=%.2f speed_gain=%.2f",
                linear, angular, delta_left, delta_right, angle_gain, speed_gain);
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr position_pub_;
};
  
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CarlControlNode>());
  rclcpp::shutdown();
  return 0;
}


