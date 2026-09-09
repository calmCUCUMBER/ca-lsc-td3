#include <cmath>
#include <memory>
#include <string>

#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/wrench.pb.h>
#include <gz/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

class GzWingForceBridge final : public rclcpp::Node
{
public:
  GzWingForceBridge()
  : Node("ca_lsc_gz_wing_force_bridge")
  {
    const auto left_topic = declare_parameter<std::string>(
      "left_gz_topic", "/ca_lsc/f3/wing_left/lift");
    const auto right_topic = declare_parameter<std::string>(
      "right_gz_topic", "/ca_lsc/f3/wing_right/lift");
    const auto elevator_topic = declare_parameter<std::string>(
      "elevator_gz_topic", "/ca_lsc/f3/elevator/joint_position");
    const auto elevator_wrench_topic = declare_parameter<std::string>(
      "elevator_wrench_gz_topic", "");
    const auto elevator_effective_angle_topic = declare_parameter<std::string>(
      "elevator_effective_angle_gz_topic", "");
    const auto elevator_dither_scale_topic = declare_parameter<std::string>(
      "elevator_dither_scale_gz_topic", "");
    left_publisher_ = create_publisher<geometry_msgs::msg::WrenchStamped>(
      "/ca_lsc/wing_lift_left_gt_enu", 10);
    right_publisher_ = create_publisher<geometry_msgs::msg::WrenchStamped>(
      "/ca_lsc/wing_lift_right_gt_enu", 10);
    elevator_publisher_ = create_publisher<std_msgs::msg::Float64>(
      "/ca_lsc/elevator_joint_position_gt", 10);
    elevator_wrench_publisher_ =
      create_publisher<geometry_msgs::msg::WrenchStamped>(
        "/ca_lsc/elevator_aero_wrench_gt_enu", 10);
    elevator_effective_angle_publisher_ =
      create_publisher<std_msgs::msg::Float64>(
        "/ca_lsc/elevator_effective_angle_gt", 10);
    bool ok = gz_node_.Subscribe(left_topic, &GzWingForceBridge::left, this) &&
      gz_node_.Subscribe(right_topic, &GzWingForceBridge::right, this) &&
      gz_node_.Subscribe(elevator_topic, &GzWingForceBridge::elevator, this);
    if (!elevator_wrench_topic.empty()) {
      ok = ok && gz_node_.Subscribe(
        elevator_wrench_topic, &GzWingForceBridge::elevatorWrench, this);
    }
    if (!elevator_effective_angle_topic.empty()) {
      ok = ok && gz_node_.Subscribe(
        elevator_effective_angle_topic,
        &GzWingForceBridge::elevatorEffectiveAngle, this);
    }
    if (!elevator_dither_scale_topic.empty()) {
      elevator_dither_scale_gz_publisher_ =
        gz_node_.Advertise<gz::msgs::Double>(elevator_dither_scale_topic);
      ok = ok && static_cast<bool>(elevator_dither_scale_gz_publisher_);
      elevator_dither_scale_subscription_ =
        create_subscription<std_msgs::msg::Float64>(
          "/ca_lsc/elevator_id_dither_scale", 10,
          [this](const std_msgs::msg::Float64 & input)
          {
            gz::msgs::Double output;
            output.set_data(input.data);
            elevator_dither_scale_gz_publisher_.Publish(output);
          });
    }
    if (!ok) {
      throw std::runtime_error("failed to subscribe to instrumented wing topics");
    }
    RCLCPP_INFO(
      get_logger(), "bridging instrumented wing lift: %s and %s; "
      "elevator joint: %s; elevator wrench: %s; effective angle: %s; "
      "dither scale: %s",
      left_topic.c_str(), right_topic.c_str(), elevator_topic.c_str(),
      elevator_wrench_topic.empty() ? "<disabled>" : elevator_wrench_topic.c_str(),
      elevator_effective_angle_topic.empty()
      ? "<disabled>" : elevator_effective_angle_topic.c_str(),
      elevator_dither_scale_topic.empty()
      ? "<disabled>" : elevator_dither_scale_topic.c_str());
  }

private:
  geometry_msgs::msg::WrenchStamped convert(const gz::msgs::Wrench & input)
  {
    geometry_msgs::msg::WrenchStamped output;
    output.header.stamp = now();
    output.header.frame_id = "world_enu";
    output.wrench.force.x = input.force().x();
    output.wrench.force.y = input.force().y();
    output.wrench.force.z = input.force().z();
    output.wrench.torque.x = input.torque().x();
    output.wrench.torque.y = input.torque().y();
    output.wrench.torque.z = input.torque().z();
    return output;
  }

  void left(const gz::msgs::Wrench & input)
  {
    left_publisher_->publish(convert(input));
  }

  void right(const gz::msgs::Wrench & input)
  {
    right_publisher_->publish(convert(input));
  }

  void elevator(const gz::msgs::Double & input)
  {
    std_msgs::msg::Float64 output;
    output.data = input.data();
    elevator_publisher_->publish(output);
  }

  void elevatorWrench(const gz::msgs::Wrench & input)
  {
    elevator_wrench_publisher_->publish(convert(input));
  }

  void elevatorEffectiveAngle(const gz::msgs::Double & input)
  {
    std_msgs::msg::Float64 output;
    output.data = input.data();
    elevator_effective_angle_publisher_->publish(output);
  }

  gz::transport::Node gz_node_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr left_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr right_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr elevator_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr
    elevator_wrench_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
    elevator_effective_angle_publisher_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr
    elevator_dither_scale_subscription_;
  gz::transport::Node::Publisher elevator_dither_scale_gz_publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GzWingForceBridge>());
  rclcpp::shutdown();
  return 0;
}
