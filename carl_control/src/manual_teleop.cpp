#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <algorithm>

class ManualTeleop : public rclcpp::Node
{
public:
  ManualTeleop() : Node("manual_teleop"), linear_speed_(0.0), steering_angle_(0.0),
                   speed_gain_(1.0), angle_gain_(1.0)
  {
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    stop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/stop_signal", 10, std::bind(&ManualTeleop::stopCallback, this, std::placeholders::_1));

    enableRawMode();
    RCLCPP_INFO(this->get_logger(), "Teleop attivo: W/S avanti-indietro, A/D sterza, R raddrizza, Q/Z velocità, E/C sterzo, Spazio stop");
    loop();
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_sub_;
  double linear_speed_;
  double steering_angle_;
  double speed_gain_;
  double angle_gain_;
  const double min_gain_ = 0.1;
  const double max_gain_ = 10.0;
  const double steering_step_ = 0.05;

  void stopCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data)
    {
      linear_speed_ = 0.0;
      steering_angle_ = 0.0;
      RCLCPP_WARN(this->get_logger(), "STOP ricevuto: azzeramento velocità");
    }
  }

  void loop()
  {
    char c;
    while (rclcpp::ok())
    {
      c = getchar();
      if (c == 'w') linear_speed_ = 1.0;
      else if (c == 's') linear_speed_ = -1.0;
      else if (c == ' ') { linear_speed_ = 0.0; steering_angle_ = 0.0; }
      else if (c == 'a') steering_angle_ += steering_step_;
      else if (c == 'd') steering_angle_ -= steering_step_;
      else if (c == 'r') steering_angle_ *= 0.9;
      else if (c == 'q') speed_gain_ = std::min(max_gain_, speed_gain_ + 0.1);
      else if (c == 'z') speed_gain_ = std::max(min_gain_, speed_gain_ - 0.1);
      else if (c == 'e') angle_gain_ = std::min(max_gain_, angle_gain_ + 0.1);
      else if (c == 'c') angle_gain_ = std::max(min_gain_, angle_gain_ - 0.1);

      steering_angle_ = std::clamp(steering_angle_, -1.0, 1.0);

      geometry_msgs::msg::Twist cmd;
      cmd.linear.x = linear_speed_ * speed_gain_;
      cmd.angular.z = steering_angle_ * angle_gain_;
      cmd_pub_->publish(cmd);
    }
  }

  void enableRawMode()
  {
    termios term;
    tcgetattr(STDIN_FILENO, &term);
    term.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &term);
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ManualTeleop>());
  rclcpp::shutdown();
  return 0;
}
