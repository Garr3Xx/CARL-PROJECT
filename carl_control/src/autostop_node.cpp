#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include <algorithm>

class AutoStopNode : public rclcpp::Node
{
public:
  AutoStopNode() : Node("autostop_node")
  {
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&AutoStopNode::scanCallback, this, std::placeholders::_1));
    
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    stop_pub_ = this->create_publisher<std_msgs::msg::Bool>("/stop_signal", 10);

    RCLCPP_INFO(this->get_logger(), "Autostop attivo con segnale esterno");
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_pub_;
  const double safety_distance_ = 0.5;

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    double min_distance = msg->range_max;
    for (const auto &range : msg->ranges)
      if (std::isfinite(range) && range < min_distance)
        min_distance = range;

    if (min_distance < safety_distance_)
    {
      geometry_msgs::msg::Twist stop_cmd;
      stop_cmd.linear.x = 0.0;
      stop_cmd.angular.z = 0.0;
      cmd_pub_->publish(stop_cmd);

      std_msgs::msg::Bool stop_msg;
      stop_msg.data = true;
      stop_pub_->publish(stop_msg);

      RCLCPP_WARN(this->get_logger(), "Ostacolo a %.2f m - STOP e segnale inviato", min_distance);
    }
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AutoStopNode>());
  rclcpp::shutdown();
  return 0;
}
