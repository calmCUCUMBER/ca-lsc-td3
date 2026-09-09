// Copyright 2026 weicheng
// SPDX-License-Identifier: Apache-2.0
//
// The aerodynamic equations in this diagnostic system follow Gazebo Sim 8
// LiftDrag (Apache-2.0):
// https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/systems/lift_drag/LiftDrag.cc
//
// Unlike an observer that replays the equations after the fact, this system
// applies the force / moment used by physics and publishes that exact
// diagnostic wrench.  It is used only by the F3 instrumented model, replacing
// selected stock LiftDrag instances.  The transition allocator remains
// unchanged.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include <gz/math/Helpers.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/wrench.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Wind.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>

namespace ca_lsc
{
class InstrumentedLiftDrag final :
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
public:
  void Configure(
    const gz::sim::Entity & entity,
    const std::shared_ptr<const sdf::Element> & sdf,
    gz::sim::EntityComponentManager &,
    gz::sim::EventManager &) override
  {
    model_ = gz::sim::Model(entity);
    sdf_ = sdf->Clone();
  }

  void PreUpdate(
    const gz::sim::UpdateInfo & info,
    gz::sim::EntityComponentManager & ecm) override
  {
    if (!initialized_) {
      initialized_ = true;
      load(ecm);
    }
    if (info.paused || !valid_) {
      return;
    }
    update(info, ecm);
  }

private:
  template<typename T>
  T parameter(const std::string & name, const T & fallback) const
  {
    return sdf_->Get<T>(name, fallback).first;
  }

  void load(gz::sim::EntityComponentManager & ecm)
  {
    if (!model_.Valid(ecm)) {
      gzerr << "InstrumentedLiftDrag must be attached to a model\n";
      return;
    }
    cla_ = parameter<double>("cla", cla_);
    cda_ = parameter<double>("cda", cda_);
    cma_ = parameter<double>("cma", cma_);
    alpha_stall_ = parameter<double>("alpha_stall", alpha_stall_);
    cla_stall_ = parameter<double>("cla_stall", cla_stall_);
    cda_stall_ = parameter<double>("cda_stall", cda_stall_);
    cma_stall_ = parameter<double>("cma_stall", cma_stall_);
    density_ = parameter<double>("air_density", density_);
    area_ = parameter<double>("area", area_);
    alpha_offset_ = parameter<double>("a0", alpha_offset_);
    cp_ = parameter<gz::math::Vector3d>("cp", cp_);
    forward_ = parameter<gz::math::Vector3d>("forward", forward_).Normalized();
    upward_ = parameter<gz::math::Vector3d>("upward", upward_).Normalized();
    control_joint_to_cl_ = parameter<double>(
      "control_joint_rad_to_cl", control_joint_to_cl_);
    cm_delta_ = parameter<double>("cm_delta", cm_delta_);
    id_dither_amplitude_rad_ = parameter<double>(
      "id_dither_amplitude_rad", id_dither_amplitude_rad_);
    id_dither_frequency_hz_ = parameter<double>(
      "id_dither_frequency_hz", id_dither_frequency_hz_);
    id_dither_phase_rad_ = parameter<double>(
      "id_dither_phase_rad", id_dither_phase_rad_);
    id_dither_scale_topic_ = parameter<std::string>(
      "id_dither_scale_topic", "");
    id_dither_scale_.store(id_dither_scale_topic_.empty() ? 1.0 : 0.0);
    publish_rate_hz_ = parameter<double>("publish_rate_hz", 100.0);
    force_topic_ = parameter<std::string>("force_topic", "");
    effective_angle_topic_ = parameter<std::string>(
      "effective_angle_topic", "");
    const auto link_name = parameter<std::string>("link_name", "");
    if (link_name.empty() || force_topic_.empty() || publish_rate_hz_ <= 0.0) {
      gzerr << "InstrumentedLiftDrag requires link_name, force_topic and a "
            << "positive publish_rate_hz\n";
      return;
    }
    const auto links = gz::sim::entitiesFromScopedName(
      link_name, ecm, model_.Entity());
    if (links.empty()) {
      gzerr << "InstrumentedLiftDrag link not found: " << link_name << "\n";
      return;
    }
    link_entity_ = *links.begin();
    if (!ecm.EntityHasComponentType(
        link_entity_, gz::sim::components::Link::typeId))
    {
      gzerr << "InstrumentedLiftDrag target is not a link\n";
      return;
    }
    const auto joint_name = parameter<std::string>("control_joint_name", "");
    if (!joint_name.empty()) {
      const auto joints = gz::sim::entitiesFromScopedName(
        joint_name, ecm, model_.Entity());
      if (joints.empty()) {
        gzerr << "InstrumentedLiftDrag joint not found: " << joint_name << "\n";
        return;
      }
      joint_entity_ = *joints.begin();
      if (!ecm.Component<gz::sim::components::JointPosition>(joint_entity_)) {
        ecm.CreateComponent(
          joint_entity_, gz::sim::components::JointPosition());
      }
    }
    gz::sim::Link(link_entity_).EnableVelocityChecks(ecm, true);
    publisher_ = node_.Advertise<gz::msgs::Wrench>(force_topic_);
    if (!publisher_) {
      gzerr << "InstrumentedLiftDrag could not advertise " << force_topic_ << "\n";
      return;
    }
    if (!effective_angle_topic_.empty()) {
      effective_angle_publisher_ =
        node_.Advertise<gz::msgs::Double>(effective_angle_topic_);
      if (!effective_angle_publisher_) {
        gzerr << "InstrumentedLiftDrag could not advertise "
              << effective_angle_topic_ << "\n";
        return;
      }
    }
    if (!id_dither_scale_topic_.empty()) {
      const std::function<void(const gz::msgs::Double &)> callback =
        [this](const gz::msgs::Double & message)
        {
          id_dither_scale_.store(
            gz::math::clamp(message.data(), 0.0, 1.0));
        };
      if (!node_.Subscribe(id_dither_scale_topic_, callback))
      {
        gzerr << "InstrumentedLiftDrag could not subscribe "
              << id_dither_scale_topic_ << "\n";
        return;
      }
    }
    publish_period_ = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    valid_ = true;
    gzmsg << "InstrumentedLiftDrag publishing applied lift on "
          << force_topic_ << "\n";
  }

  void update(
    const gz::sim::UpdateInfo & info,
    gz::sim::EntityComponentManager & ecm)
  {
    const auto * linear = ecm.Component<
      gz::sim::components::WorldLinearVelocity>(link_entity_);
    const auto * angular = ecm.Component<
      gz::sim::components::WorldAngularVelocity>(link_entity_);
    const auto * pose_component = ecm.Component<
      gz::sim::components::WorldPose>(link_entity_);
    if (!linear || !angular || !pose_component) {
      return;
    }
    const auto & pose = pose_component->Data();
    const auto cp_world = pose.Rot().RotateVector(cp_);
    auto velocity = linear->Data() + angular->Data().Cross(cp_world);
    const auto wind_entity = ecm.EntityByComponents(gz::sim::components::Wind());
    if (wind_entity != gz::sim::kNullEntity) {
      const auto * wind = ecm.Component<
        gz::sim::components::WorldLinearVelocity>(wind_entity);
      if (wind) {
        velocity -= wind->Data();
      }
    }
    if (velocity.Length() <= 0.01) {
      return;
    }
    const auto velocity_unit = velocity.Normalized();
    const auto forward_world = pose.Rot().RotateVector(forward_);
    if (forward_world.Dot(velocity) <= 0.0) {
      return;
    }
    const auto upward_world = pose.Rot().RotateVector(upward_);
    const auto span_world = forward_world.Cross(upward_world).Normalized();
    const double sin_sweep = gz::math::clamp(
      span_world.Dot(velocity_unit), -1.0, 1.0);
    const double cos2_sweep = 1.0 - sin_sweep * sin_sweep;
    const auto velocity_plane = velocity - velocity.Dot(span_world) * span_world;
    if (velocity_plane.Length() <= 0.01) {
      return;
    }
    const auto drag_direction = -velocity_plane.Normalized();
    const auto lift_direction = span_world.Cross(velocity_plane).Normalized();
    const double cos_alpha = gz::math::clamp(
      lift_direction.Dot(upward_world), -1.0, 1.0);
    double alpha = alpha_offset_ - std::acos(cos_alpha);
    if (lift_direction.Dot(forward_world) >= 0.0) {
      alpha = alpha_offset_ + std::acos(cos_alpha);
    }
    while (std::abs(alpha) > 0.5 * GZ_PI) {
      alpha = alpha > 0.0 ? alpha - GZ_PI : alpha + GZ_PI;
    }
    const double speed = velocity_plane.Length();
    const double dynamic_pressure = 0.5 * density_ * speed * speed;
    double control_angle = 0.0;
    if (joint_entity_ != gz::sim::kNullEntity) {
      const auto * joint = ecm.Component<
        gz::sim::components::JointPosition>(joint_entity_);
      if (joint && !joint->Data().empty()) {
        control_angle = joint->Data()[0];
      }
    }
    if (
      id_dither_amplitude_rad_ != 0.0
      && id_dither_frequency_hz_ > 0.0
    ) {
      const double time_s = std::chrono::duration<double>(
        info.simTime).count();
      control_angle += id_dither_scale_.load() * id_dither_amplitude_rad_
        * std::sin(
        2.0 * GZ_PI * id_dither_frequency_hz_ * time_s
        + id_dither_phase_rad_);
    }

    double cl = coefficient(alpha, cla_, cla_stall_, true) * cos2_sweep;
    if (joint_entity_ != gz::sim::kNullEntity) {
      cl += control_joint_to_cl_ * control_angle;
    }
    const auto lift = cl * dynamic_pressure * area_ * lift_direction;
    const double cd = std::abs(
      coefficient(alpha, cda_, cda_stall_, false) * cos2_sweep);
    const auto drag = cd * dynamic_pressure * area_ * drag_direction;
    double cm = coefficient(alpha, cma_, cma_stall_, true) * cos2_sweep;
    if (joint_entity_ != gz::sim::kNullEntity) {
      cm += cm_delta_ * control_angle;
    }
    const auto intrinsic_moment = cm * dynamic_pressure * area_ * span_world;
    const auto force = lift + drag;
    const auto total_torque = intrinsic_moment + cp_world.Cross(force);
    gz::sim::Link(link_entity_).AddWorldWrench(ecm, force, total_torque);

    if (info.simTime - last_publish_time_ < publish_period_) {
      return;
    }
    last_publish_time_ = info.simTime;
    gz::msgs::Wrench message;
    message.mutable_force()->set_x(lift.X());
    message.mutable_force()->set_y(lift.Y());
    message.mutable_force()->set_z(lift.Z());
    message.mutable_torque()->set_x(total_torque.X());
    message.mutable_torque()->set_y(total_torque.Y());
    message.mutable_torque()->set_z(total_torque.Z());
    publisher_.Publish(message);
    if (effective_angle_publisher_) {
      gz::msgs::Double angle_message;
      angle_message.set_data(control_angle);
      effective_angle_publisher_.Publish(angle_message);
    }
  }

  double coefficient(
    double alpha, double pre_stall_slope, double post_stall_slope,
    bool sign_clamp) const
  {
    if (alpha > alpha_stall_) {
      const double value = pre_stall_slope * alpha_stall_
        + post_stall_slope * (alpha - alpha_stall_);
      return sign_clamp ? std::max(0.0, value) : value;
    }
    if (alpha < -alpha_stall_) {
      const double value = -pre_stall_slope * alpha_stall_
        + post_stall_slope * (alpha + alpha_stall_);
      return sign_clamp ? std::min(0.0, value) : value;
    }
    return pre_stall_slope * alpha;
  }

  gz::sim::Model model_{gz::sim::kNullEntity};
  gz::sim::Entity link_entity_{gz::sim::kNullEntity};
  gz::sim::Entity joint_entity_{gz::sim::kNullEntity};
  std::shared_ptr<sdf::Element> sdf_;
  gz::transport::Node node_;
  gz::transport::Node::Publisher publisher_;
  gz::transport::Node::Publisher effective_angle_publisher_;
  std::string force_topic_;
  std::string effective_angle_topic_;
  std::string id_dither_scale_topic_;
  gz::math::Vector3d cp_{0.0, 0.0, 0.0};
  gz::math::Vector3d forward_{1.0, 0.0, 0.0};
  gz::math::Vector3d upward_{0.0, 0.0, 1.0};
  double cla_{1.0};
  double cda_{0.01};
  double cma_{0.0};
  double alpha_stall_{GZ_PI_2};
  double cla_stall_{0.0};
  double cda_stall_{1.0};
  double cma_stall_{0.0};
  double cm_delta_{0.0};
  double id_dither_amplitude_rad_{0.0};
  double id_dither_frequency_hz_{0.0};
  double id_dither_phase_rad_{0.0};
  std::atomic<double> id_dither_scale_{1.0};
  double density_{1.2041};
  double area_{1.0};
  double alpha_offset_{0.0};
  double control_joint_to_cl_{4.0};
  double publish_rate_hz_{100.0};
  std::chrono::duration<double> publish_period_{0.01};
  std::chrono::steady_clock::duration last_publish_time_{};
  bool initialized_{false};
  bool valid_{false};
};

// Read-only joint telemetry for the F3 control-authority calculation.  This
// system does not command the joint and does not participate in allocation.
class JointPositionTelemetry final :
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
public:
  void Configure(
    const gz::sim::Entity & entity,
    const std::shared_ptr<const sdf::Element> & sdf,
    gz::sim::EntityComponentManager &,
    gz::sim::EventManager &) override
  {
    model_ = gz::sim::Model(entity);
    sdf_ = sdf->Clone();
  }

  void PreUpdate(
    const gz::sim::UpdateInfo & info,
    gz::sim::EntityComponentManager & ecm) override
  {
    if (!initialized_) {
      initialized_ = true;
      load(ecm);
    }
    if (info.paused || !valid_) {
      return;
    }
    const auto * position = ecm.Component<
      gz::sim::components::JointPosition>(joint_entity_);
    if (!position || position->Data().empty()) {
      return;
    }
    if (info.simTime - last_publish_time_ < publish_period_) {
      return;
    }
    last_publish_time_ = info.simTime;
    gz::msgs::Double message;
    message.set_data(position->Data()[0]);
    publisher_.Publish(message);
  }

private:
  void load(gz::sim::EntityComponentManager & ecm)
  {
    const auto joint_name = sdf_->Get<std::string>("joint_name", "").first;
    const auto topic = sdf_->Get<std::string>("topic", "").first;
    const double publish_rate_hz =
      sdf_->Get<double>("publish_rate_hz", 100.0).first;
    if (!model_.Valid(ecm) || joint_name.empty() || topic.empty() ||
      publish_rate_hz <= 0.0)
    {
      gzerr << "JointPositionTelemetry requires a valid model, joint_name, "
            << "topic and positive publish_rate_hz\n";
      return;
    }
    const auto joints = gz::sim::entitiesFromScopedName(
      joint_name, ecm, model_.Entity());
    if (joints.empty()) {
      gzerr << "JointPositionTelemetry joint not found: " << joint_name << "\n";
      return;
    }
    joint_entity_ = *joints.begin();
    if (!ecm.Component<gz::sim::components::JointPosition>(joint_entity_)) {
      ecm.CreateComponent(
        joint_entity_, gz::sim::components::JointPosition());
    }
    publisher_ = node_.Advertise<gz::msgs::Double>(topic);
    if (!publisher_) {
      gzerr << "JointPositionTelemetry could not advertise " << topic << "\n";
      return;
    }
    publish_period_ = std::chrono::duration<double>(1.0 / publish_rate_hz);
    valid_ = true;
    gzmsg << "JointPositionTelemetry publishing " << joint_name << " on "
          << topic << "\n";
  }

  gz::sim::Model model_{gz::sim::kNullEntity};
  gz::sim::Entity joint_entity_{gz::sim::kNullEntity};
  std::shared_ptr<sdf::Element> sdf_;
  gz::transport::Node node_;
  gz::transport::Node::Publisher publisher_;
  std::chrono::duration<double> publish_period_{0.01};
  std::chrono::steady_clock::duration last_publish_time_{};
  bool initialized_{false};
  bool valid_{false};
};
}  // namespace ca_lsc

GZ_ADD_PLUGIN(
  ca_lsc::InstrumentedLiftDrag,
  gz::sim::System,
  ca_lsc::InstrumentedLiftDrag::ISystemConfigure,
  ca_lsc::InstrumentedLiftDrag::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(
  ca_lsc::InstrumentedLiftDrag,
  "ca_lsc::InstrumentedLiftDrag")

GZ_ADD_PLUGIN(
  ca_lsc::JointPositionTelemetry,
  gz::sim::System,
  ca_lsc::JointPositionTelemetry::ISystemConfigure,
  ca_lsc::JointPositionTelemetry::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(
  ca_lsc::JointPositionTelemetry,
  "ca_lsc::JointPositionTelemetry")
