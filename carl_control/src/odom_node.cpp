#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include <cmath>

class OdomPublisherNode : public rclcpp::Node
{
public:
  OdomPublisherNode() : Node("odom_publisher_node"), x_(0.0), y_(0.0), theta_(0.0)
  {
    declare_parameter("wheel_radius", 0.075);
    declare_parameter("wheel_separation", 0.31);
    declare_parameter("steering_gain", 1.0);
    declare_parameter("publish_odom", true);

    get_parameter("wheel_radius", wheel_radius_);
    get_parameter("wheel_separation", wheel_separation_);
    get_parameter("steering_gain", steering_gain_);
    get_parameter("publish_odom", publish_odom_);

    joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, std::bind(&OdomPublisherNode::jointCallback, this, std::placeholders::_1));

    if (publish_odom_)
    {
      odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
      tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }

    last_time_ = this->now();
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  double wheel_radius_;
  double wheel_separation_;
  double steering_gain_;
  bool publish_odom_;

  double last_left_pos_ = 0.0;
  double last_right_pos_ = 0.0;
  bool first_read_ = true;

  double x_, y_, theta_;
  rclcpp::Time last_time_;

  void jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    double left_pos = 0.0;
    double right_pos = 0.0;
    double steer_left = 0.0;
    double steer_right = 0.0;

    for (size_t i = 0; i < msg->name.size(); ++i)
    {
      if (msg->name[i] == "Ruota_sinistra_anteriore" || msg->name[i] == "Ruota_sinistra_posteriore")
        left_pos += msg->position[i] / 2.0;

      if (msg->name[i] == "Ruota_destra_anteriore" || msg->name[i] == "Ruota_destra_posteriore")
        right_pos += msg->position[i] / 2.0;

      if (msg->name[i] == "Inclina_sinistra_anteriore")
        steer_left = msg->position[i];

      if (msg->name[i] == "Inclina_destra_anteriore")
        steer_right = msg->position[i];
    }

    if (first_read_)
    {
      last_left_pos_ = left_pos;
      last_right_pos_ = right_pos;
      last_time_ = this->now();
      first_read_ = false;
      return;
    }

    double delta_left = left_pos - last_left_pos_;
    double delta_right = right_pos - last_right_pos_;
    last_left_pos_ = left_pos;
    last_right_pos_ = right_pos;

    double delta_s = wheel_radius_ * (delta_right + delta_left) / 2.0;
    double steering_angle = (steer_left + steer_right) / 2.0 * steering_gain_;
    double delta_theta = (delta_s / wheel_separation_) * tan(steering_angle);

    double dt = (this->now() - last_time_).seconds();
    last_time_ = this->now();

    double vx = delta_s / dt;
    double vth = delta_theta / dt;

    theta_ += delta_theta;
    x_ += delta_s * cos(theta_);
    y_ += delta_s * sin(theta_);

    if (publish_odom_)
    {
      auto odom_msg = nav_msgs::msg::Odometry();
      odom_msg.header.stamp = this->now();
      odom_msg.header.frame_id = "odom";
      odom_msg.child_frame_id = "base_link";

      odom_msg.pose.pose.position.x = -y_;
      odom_msg.pose.pose.position.y = x_;

      tf2::Quaternion q;
      q.setRPY(0, 0, theta_);
      odom_msg.pose.pose.orientation.x = q.x();
      odom_msg.pose.pose.orientation.y = q.y();
      odom_msg.pose.pose.orientation.z = q.z();
      odom_msg.pose.pose.orientation.w = q.w();

      odom_msg.twist.twist.linear.x = vx;
      odom_msg.twist.twist.angular.z = vth;

      odom_pub_->publish(odom_msg);

      geometry_msgs::msg::TransformStamped tf_msg;
      tf_msg.header.stamp = this->now();
      tf_msg.header.frame_id = "odom";
      tf_msg.child_frame_id = "base_link";

      tf_msg.transform.translation.x = -y_;
      tf_msg.transform.translation.y = x_;
      tf_msg.transform.translation.z = 0.0;
      tf_msg.transform.rotation.x = q.x();
      tf_msg.transform.rotation.y = q.y();
      tf_msg.transform.rotation.z = q.z();
      tf_msg.transform.rotation.w = q.w();

      tf_broadcaster_->sendTransform(tf_msg);
    }
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomPublisherNode>());
  rclcpp::shutdown();
  return 0;
}
