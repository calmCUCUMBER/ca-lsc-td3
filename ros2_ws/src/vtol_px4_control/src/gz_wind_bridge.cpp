#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <gz/msgs/wind.pb.h>
#include <gz/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

class GzWindBridge final : public rclcpp::Node
{
public:
  GzWindBridge()
  : Node("ca_lsc_gz_wind_bridge")
  {
    const auto gz_topic = declare_parameter<std::string>(
      "gz_topic", "/world/default/wind/");
    command_timeout_s_ = declare_parameter<double>("command_timeout_s", 0.25);
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 50.0);
    if (!std::isfinite(command_timeout_s_) || command_timeout_s_ <= 0.0 ||
      !std::isfinite(publish_rate_hz_) || publish_rate_hz_ <= 0.0)
    {
      throw std::runtime_error("invalid wind bridge timing parameters");
    }

    gz_publisher_ = gz_node_.Advertise<gz::msgs::Wind>(gz_topic);
    if (!gz_publisher_)
    {
      throw std::runtime_error("failed to advertise Gazebo wind topic " + gz_topic);
    }
    status_publisher_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
      "/ca_lsc/gust_status_enu", 10);
    command_subscription_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
      "/ca_lsc/gust_command_enu", 10,
      [this](geometry_msgs::msg::Vector3Stamped::SharedPtr message) {
        if (!std::isfinite(message->vector.x) ||
          !std::isfinite(message->vector.y) ||
          !std::isfinite(message->vector.z))
        {
          RCLCPP_WARN(get_logger(), "rejected non-finite gust command");
          return;
        }
        command_ = *message;
        command_received_ = true;
        last_command_time_ = std::chrono::steady_clock::now();
      });
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate_hz_),
      std::bind(&GzWindBridge::publish, this));
    RCLCPP_INFO(get_logger(), "bridging ROS gust commands to %s", gz_topic.c_str());
  }

private:
  void publish()
  {
    if (!command_received_) {
      return;
    }
    const double age_s = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - last_command_time_).count();
    geometry_msgs::msg::Vector3Stamped applied = command_;
    if (age_s > command_timeout_s_) {
      applied.vector.x = 0.0;
      applied.vector.y = 0.0;
      applied.vector.z = 0.0;
    }

    gz::msgs::Wind message;
    message.set_enable_wind(true);
    auto * velocity = message.mutable_linear_velocity();
    velocity->set_x(applied.vector.x);
    velocity->set_y(applied.vector.y);
    velocity->set_z(applied.vector.z);
    if (!gz_publisher_.Publish(message)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Gazebo wind publish failed (is the matching GZ_PARTITION active?)");
      return;
    }
    applied.header.stamp = now();
    applied.header.frame_id = "world_enu";
    status_publisher_->publish(applied);
  }

  gz::transport::Node gz_node_;
  gz::transport::Node::Publisher gz_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr
    command_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr
    status_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  geometry_msgs::msg::Vector3Stamped command_;
  std::chrono::steady_clock::time_point last_command_time_;
  bool command_received_{false};
  double command_timeout_s_{0.25};
  double publish_rate_hz_{50.0};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GzWindBridge>());
  rclcpp::shutdown();
  return 0;
}
