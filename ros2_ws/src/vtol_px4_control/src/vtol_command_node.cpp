// Copyright 2026 Weicheng

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <std_srvs/srv/trigger.hpp>

using namespace std::chrono_literals;

namespace
{

constexpr float kNan = std::numeric_limits<float>::quiet_NaN();
constexpr float kMavVtolStateMc = 3.0F;
constexpr float kMavVtolStateFw = 4.0F;

double clamp_symmetric(const double value, const double limit)
{
    return std::clamp(value, -limit, limit);
}

std::string upper_copy(std::string value)
{
    std::transform(
        value.begin(), value.end(), value.begin(),
        [](const unsigned char character) {
            return static_cast<char>(std::toupper(character));
        });
    return value;
}

std::optional<uint64_t> parse_action_sequence(
    const std::string &frame_id)
{
    std::string value = frame_id;
    constexpr const char *prefix = "seq=";
    if (value.rfind(prefix, 0) == 0) {
        value = value.substr(std::string(prefix).size());
    }
    if (value.empty()) {
        return std::nullopt;
    }
    try {
        size_t consumed = 0;
        const auto sequence = std::stoull(value, &consumed, 10);
        if (consumed != value.size()) {
            return std::nullopt;
        }
        return sequence;
    } catch (const std::exception &) {
        return std::nullopt;
    }
}

}  // namespace

class VtolCommandNode : public rclcpp::Node
{
public:
    VtolCommandNode()
    : Node("vtol_command_node")
    {
        auto_start_mission_ =
            declare_parameter<bool>("auto_start_mission", false);
        takeoff_altitude_m_ =
            declare_parameter<double>("takeoff_altitude_m", 20.0);
        altitude_tolerance_m_ =
            declare_parameter<double>("altitude_tolerance_m", 1.0);
        takeoff_completion_tolerance_m_ =
            declare_parameter<double>("takeoff_completion_tolerance_m", 0.5);
        takeoff_completion_vertical_speed_tolerance_mps_ =
            declare_parameter<double>(
                "takeoff_completion_vertical_speed_tolerance_mps", 0.3);
        preflight_stream_s_ =
            declare_parameter<double>("preflight_stream_s", 1.5);
        hold_mc_s_ = declare_parameter<double>("hold_mc_s", 4.0);
        require_transition_stability_ =
            declare_parameter<bool>(
                "require_transition_stability", false);
        transition_stability_dwell_s_ =
            declare_parameter<double>(
                "transition_stability_dwell_s", 2.0);
        transition_altitude_tolerance_m_ =
            declare_parameter<double>(
                "transition_altitude_tolerance_m", 1.0);
        transition_vertical_speed_tolerance_mps_ =
            declare_parameter<double>(
                "transition_vertical_speed_tolerance_mps", 0.2);
        transition_groundspeed_tolerance_mps_ =
            declare_parameter<double>(
                "transition_groundspeed_tolerance_mps", 0.2);
        hold_mc_timeout_s_ =
            declare_parameter<double>("hold_mc_timeout_s", 45.0);
        hold_fw_s_ = declare_parameter<double>("hold_fw_s", 12.0);
        hold_after_mc_s_ =
            declare_parameter<double>("hold_after_mc_s", 3.0);
        forward_distance_m_ =
            declare_parameter<double>("forward_distance_m", 60.0);
        max_horizontal_speed_mps_ =
            declare_parameter<double>("max_horizontal_speed_mps", 15.0);
        max_vertical_speed_mps_ =
            declare_parameter<double>("max_vertical_speed_mps", 1.5);
        max_yaw_rate_rps_ =
            declare_parameter<double>("max_yaw_rate_rps", 0.25);
        fw_min_forward_speed_mps_ =
            declare_parameter<double>("fw_min_forward_speed_mps", 15.0);
        action_timeout_s_ =
            declare_parameter<double>("action_timeout_s", 0.5);
        require_a0_rl_action_ready_ =
            declare_parameter<bool>("require_a0_rl_action_ready", false);
        a0_rl_action_ready_timeout_s_ =
            declare_parameter<double>("a0_rl_action_ready_timeout_s", 0.5);
        command_retry_s_ =
            declare_parameter<double>("command_retry_s", 1.0);
        max_command_attempts_ =
            declare_parameter<int>("max_command_attempts", 5);
        ready_timeout_s_ =
            declare_parameter<double>("ready_timeout_s", 45.0);
        takeoff_timeout_s_ =
            declare_parameter<double>("takeoff_timeout_s", 45.0);
        transition_timeout_s_ =
            declare_parameter<double>("transition_timeout_s", 25.0);
        landing_timeout_s_ =
            declare_parameter<double>("landing_timeout_s", 75.0);

        if (hold_mc_s_ < 0.0 ||
            altitude_tolerance_m_ <= 0.0 ||
            takeoff_completion_tolerance_m_ <= 0.0 ||
            takeoff_completion_vertical_speed_tolerance_mps_ <= 0.0 ||
            transition_stability_dwell_s_ <= 0.0 ||
            transition_altitude_tolerance_m_ <= 0.0 ||
            transition_vertical_speed_tolerance_mps_ <= 0.0 ||
            transition_groundspeed_tolerance_mps_ <= 0.0 ||
            a0_rl_action_ready_timeout_s_ <= 0.0 ||
            hold_mc_timeout_s_ <= hold_mc_s_) {
            throw std::invalid_argument(
                "invalid multicopter hold/stability-gate parameters");
        }

        const auto sensor_qos =
            rclcpp::SensorDataQoS().keep_last(10);

        status_sub_ =
            create_subscription<px4_msgs::msg::VehicleStatus>(
                "/fmu/out/vehicle_status", sensor_qos,
                std::bind(
                    &VtolCommandNode::status_callback, this,
                    std::placeholders::_1));
        position_sub_ =
            create_subscription<px4_msgs::msg::VehicleLocalPosition>(
                "/fmu/out/vehicle_local_position", sensor_qos,
                std::bind(
                    &VtolCommandNode::position_callback, this,
                    std::placeholders::_1));
        ack_sub_ =
            create_subscription<px4_msgs::msg::VehicleCommandAck>(
                "/fmu/out/vehicle_command_ack", sensor_qos,
                std::bind(
                    &VtolCommandNode::ack_callback, this,
                    std::placeholders::_1));
        request_sub_ =
            create_subscription<std_msgs::msg::String>(
                "/vtol/command/request", 10,
                std::bind(
                    &VtolCommandNode::request_callback, this,
                    std::placeholders::_1));
        velocity_sub_ =
            create_subscription<geometry_msgs::msg::Twist>(
                "/vtol/command/velocity_setpoint_ned", 10,
                std::bind(
                    &VtolCommandNode::velocity_callback, this,
                    std::placeholders::_1));
        stamped_velocity_sub_ =
            create_subscription<geometry_msgs::msg::TwistStamped>(
                "/vtol/command/velocity_setpoint_stamped", 10,
                std::bind(
                    &VtolCommandNode::stamped_velocity_callback, this,
                    std::placeholders::_1));
        a0_rl_action_sub_ =
            create_subscription<std_msgs::msg::String>(
                "/ca_lsc/a0_rl_action", 10,
                std::bind(
                    &VtolCommandNode::a0_rl_action_callback, this,
                    std::placeholders::_1));

        vehicle_command_pub_ =
            create_publisher<px4_msgs::msg::VehicleCommand>(
                "/fmu/in/vehicle_command", 10);
        offboard_mode_pub_ =
            create_publisher<px4_msgs::msg::OffboardControlMode>(
                "/fmu/in/offboard_control_mode", 10);
        trajectory_pub_ =
            create_publisher<px4_msgs::msg::TrajectorySetpoint>(
                "/fmu/in/trajectory_setpoint", 10);
        state_pub_ =
            create_publisher<std_msgs::msg::String>(
                "/vtol/command/state", rclcpp::QoS(1).transient_local());
        action_ack_pub_ =
            create_publisher<std_msgs::msg::UInt64>(
                "/vtol/command/action_ack", 10);

        start_service_ =
            create_service<std_srvs::srv::Trigger>(
                "/vtol/command/start_mission",
                std::bind(
                    &VtolCommandNode::start_service_callback, this,
                    std::placeholders::_1, std::placeholders::_2));
        abort_service_ =
            create_service<std_srvs::srv::Trigger>(
                "/vtol/command/abort",
                std::bind(
                    &VtolCommandNode::abort_service_callback, this,
                    std::placeholders::_1, std::placeholders::_2));

        const double command_period_ms =
            declare_parameter<double>("command_period_ms", 50.0);
        command_period_ = std::chrono::duration<double, std::milli>(
            std::clamp(command_period_ms, 5.0, 100.0));
        timer_ = rclcpp::create_timer(
            this,
            get_clock(),
            rclcpp::Duration(command_period_),
            std::bind(&VtolCommandNode::timer_callback, this));

        const auto startup_time = now();
        state_entered_at_ = startup_time;
        request_started_at_ = startup_time;
        velocity_received_at_ = startup_time;
        last_state_publish_at_ = startup_time;
        publish_state();

        if (auto_start_mission_) {
            start_requested_ = true;
            sequence_enabled_ = true;
            request_started_at_ = now();
        }

        RCLCPP_INFO(
            get_logger(),
            "VTOL command node ready; auto mission=%s; "
            "command period=%.2f ms",
            auto_start_mission_ ? "true" : "false",
            command_period_.count());
    }

private:
    enum class State
    {
        Idle,
        WaitReady,
        PreflightStream,
        RequestOffboard,
        RequestArm,
        Takeoff,
        HoldMc,
        TransitionFw,
        HoldFw,
        TransitionMc,
        HoldAfterMc,
        Landing,
        Completed,
        Aborted,
        Error,
    };

    struct PendingCommand
    {
        uint16_t command{};
        int attempts{};
        rclcpp::Time last_sent{};
        bool accepted{};
    };

    struct StabilityDiagnostics
    {
        bool valid{false};
        double altitude_relative_m{std::numeric_limits<double>::quiet_NaN()};
        double altitude_error_m{std::numeric_limits<double>::quiet_NaN()};
        double vertical_speed_up_mps{std::numeric_limits<double>::quiet_NaN()};
        double groundspeed_mps{std::numeric_limits<double>::quiet_NaN()};
        bool height_ok{false};
        bool vertical_speed_ok{false};
        bool groundspeed_ok{false};
        bool gate_ok{false};
    };

    static const char *state_name(const State state)
    {
        switch (state) {
        case State::Idle:
            return "IDLE";
        case State::WaitReady:
            return "WAIT_READY";
        case State::PreflightStream:
            return "PREFLIGHT_STREAM";
        case State::RequestOffboard:
            return "REQUEST_OFFBOARD";
        case State::RequestArm:
            return "REQUEST_ARM";
        case State::Takeoff:
            return "TAKEOFF";
        case State::HoldMc:
            return "HOLD_MC";
        case State::TransitionFw:
            return "TRANSITION_FW";
        case State::HoldFw:
            return "HOLD_FW";
        case State::TransitionMc:
            return "TRANSITION_MC";
        case State::HoldAfterMc:
            return "HOLD_AFTER_MC";
        case State::Landing:
            return "LANDING";
        case State::Completed:
            return "COMPLETED";
        case State::Aborted:
            return "ABORTED";
        case State::Error:
            return "ERROR";
        }
        return "UNKNOWN";
    }

    uint64_t timestamp_us() const
    {
        return static_cast<uint64_t>(now().nanoseconds() / 1000);
    }

    double state_elapsed_s() const
    {
        return (now() - state_entered_at_).seconds();
    }

    bool position_is_valid() const
    {
        return position_received_ &&
               local_position_.xy_valid &&
               local_position_.z_valid &&
               std::isfinite(local_position_.x) &&
               std::isfinite(local_position_.y) &&
               std::isfinite(local_position_.z);
    }

    bool vehicle_is_ready() const
    {
        return status_received_ &&
               position_is_valid() &&
               vehicle_status_.is_vtol &&
               vehicle_status_.pre_flight_checks_pass &&
               !vehicle_status_.failsafe;
    }

    bool is_armed() const
    {
        return status_received_ &&
               vehicle_status_.arming_state ==
               px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
    }

    bool is_fixed_wing() const
    {
        return status_received_ &&
               vehicle_status_.vehicle_type ==
               px4_msgs::msg::VehicleStatus::VEHICLE_TYPE_FIXED_WING;
    }

    bool is_multicopter() const
    {
        return status_received_ &&
               vehicle_status_.vehicle_type ==
               px4_msgs::msg::VehicleStatus::VEHICLE_TYPE_ROTARY_WING &&
               !vehicle_status_.in_transition_mode;
    }

    bool offboard_is_active() const
    {
        return status_received_ &&
               vehicle_status_.nav_state ==
               px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
    }

    void set_state(const State next_state, const std::string &reason)
    {
        if (state_ == next_state) {
            return;
        }

        RCLCPP_INFO(
            get_logger(), "STATE %s -> %s | %s",
            state_name(state_), state_name(next_state), reason.c_str());
        state_ = next_state;
        state_entered_at_ = now();
        pending_command_.reset();

        if (next_state == State::HoldMc) {
            capture_hold_position(true);
        }
        if (next_state == State::HoldAfterMc) {
            capture_hold_position();
        }
        if (next_state == State::HoldMc) {
            transition_stable_since_.reset();
        }
        if (next_state == State::TransitionMc) {
            capture_hold_position();
        }

        publish_state();
    }

    void publish_state()
    {
        std_msgs::msg::String message;
        std::ostringstream stream;
        stream << state_name(state_);
        if (home_valid_ && position_is_valid()) {
            const StabilityDiagnostics diagnostics =
                transition_stability_diagnostics();
            stream << "|altitude_datum_local_m="
                   << -static_cast<double>(home_z_)
                   << "|altitude_local_m="
                   << -static_cast<double>(local_position_.z)
                   << "|altitude_relative_m="
                   << diagnostics.altitude_relative_m
                   << "|target_altitude_relative_m="
                   << takeoff_altitude_m_
                   << "|target_altitude_local_m="
                   << -static_cast<double>(target_z_)
                   << "|altitude_error_m="
                   << diagnostics.altitude_error_m
                   << "|vz_up_mps="
                   << diagnostics.vertical_speed_up_mps
                   << "|groundspeed_mps="
                   << diagnostics.groundspeed_mps
                   << "|height_gate_ok="
                   << (diagnostics.height_ok ? 1 : 0)
                   << "|vz_gate_ok="
                   << (diagnostics.vertical_speed_ok ? 1 : 0)
                   << "|groundspeed_gate_ok="
                   << (diagnostics.groundspeed_ok ? 1 : 0)
                   << "|stability_gate_ok="
                   << (diagnostics.gate_ok ? 1 : 0)
                   << "|stability_dwell_s="
                   << transition_stability_dwell_s()
                   << "|stability_reset_count="
                   << transition_stability_reset_count_;
            if (require_a0_rl_action_ready_) {
                stream << "|a0_rl_action_ready="
                       << (a0_rl_action_is_ready() ? 1 : 0)
                       << "|a0_rl_action_age_s="
                       << a0_rl_action_age_s();
            }
        }
        if (!last_error_.empty()) {
            stream << "|error=" << last_error_;
        }
        message.data = stream.str();
        state_pub_->publish(message);
    }

    void capture_home()
    {
        home_x_ = local_position_.x;
        home_y_ = local_position_.y;
        home_z_ = local_position_.z;
        home_heading_ =
            std::isfinite(local_position_.heading)
            ? local_position_.heading : 0.0F;
        home_valid_ = true;
        target_z_ = static_cast<float>(
            static_cast<double>(home_z_) - takeoff_altitude_m_);
        capture_hold_position(true);

        const double heading = static_cast<double>(home_heading_);
        fw_target_x_ = static_cast<float>(
            static_cast<double>(home_x_) +
            forward_distance_m_ * std::cos(heading));
        fw_target_y_ = static_cast<float>(
            static_cast<double>(home_y_) +
            forward_distance_m_ * std::sin(heading));
    }

    void capture_hold_position(const bool use_target_altitude = false)
    {
        if (!position_is_valid()) {
            return;
        }
        hold_x_ = local_position_.x;
        hold_y_ = local_position_.y;
        hold_z_ = use_target_altitude ? target_z_ : local_position_.z;
        hold_heading_ =
            std::isfinite(local_position_.heading)
            ? local_position_.heading : home_heading_;
    }

    void begin_prepare(const bool run_full_sequence)
    {
        if (state_ != State::Idle &&
            state_ != State::Completed &&
            state_ != State::Aborted &&
            state_ != State::Error) {
            RCLCPP_WARN(
                get_logger(), "PREPARE ignored while in %s",
                state_name(state_));
            return;
        }
        if (is_armed()) {
            RCLCPP_ERROR(
                get_logger(),
                "Cannot begin a new sequence while PX4 is armed");
            return;
        }

        last_error_.clear();
        terminal_error_pending_ = false;
        abort_pending_ = false;
        land_after_transition_ = false;
        sequence_enabled_ = run_full_sequence;
        start_requested_ = true;
        request_started_at_ = now();
        set_state(State::WaitReady, "prepare requested");
    }

    void request_transition_fw()
    {
        if (state_ == State::TransitionFw) {
            return;
        }
        if ((state_ != State::HoldMc && state_ != State::HoldAfterMc) ||
            !is_multicopter()) {
            RCLCPP_WARN(
                get_logger(),
                "TRANSITION_FW requires a multicopter hold state");
            return;
        }
        if (require_transition_stability_ &&
            transition_stability_dwell_s() <
                transition_stability_dwell_s_) {
            RCLCPP_WARN(
                get_logger(),
                "TRANSITION_FW rejected: nominal stability gate is not ready");
            return;
        }
        set_state(State::TransitionFw, "forward transition requested");
    }

    StabilityDiagnostics transition_stability_diagnostics() const
    {
        StabilityDiagnostics diagnostics{};
        if (!home_valid_ || !position_is_valid()) {
            return diagnostics;
        }

        diagnostics.valid = true;
        diagnostics.altitude_relative_m =
            static_cast<double>(home_z_ - local_position_.z);
        diagnostics.altitude_error_m =
            diagnostics.altitude_relative_m - takeoff_altitude_m_;
        diagnostics.height_ok =
            std::abs(diagnostics.altitude_error_m) <=
            transition_altitude_tolerance_m_;

        if (!local_position_.v_xy_valid || !local_position_.v_z_valid ||
            !std::isfinite(local_position_.vx) ||
            !std::isfinite(local_position_.vy) ||
            !std::isfinite(local_position_.vz)) {
            return diagnostics;
        }

        diagnostics.vertical_speed_up_mps =
            -static_cast<double>(local_position_.vz);
        diagnostics.groundspeed_mps = std::hypot(
            static_cast<double>(local_position_.vx),
            static_cast<double>(local_position_.vy));
        diagnostics.vertical_speed_ok =
            std::abs(diagnostics.vertical_speed_up_mps) <=
            transition_vertical_speed_tolerance_mps_;
        diagnostics.groundspeed_ok =
            diagnostics.groundspeed_mps <=
            transition_groundspeed_tolerance_mps_;
        diagnostics.gate_ok =
            diagnostics.height_ok &&
            diagnostics.vertical_speed_ok &&
            diagnostics.groundspeed_ok;
        return diagnostics;
    }

    bool transition_stability_conditions_met() const
    {
        return transition_stability_diagnostics().gate_ok;
    }

    double a0_rl_action_age_s() const
    {
        if (!a0_rl_action_received_at_.has_value()) {
            return std::numeric_limits<double>::quiet_NaN();
        }
        return (now() - *a0_rl_action_received_at_).seconds();
    }

    bool a0_rl_action_is_ready() const
    {
        if (!require_a0_rl_action_ready_) {
            return true;
        }
        const double age_s = a0_rl_action_age_s();
        return std::isfinite(age_s) &&
               age_s >= 0.0 &&
               age_s <= a0_rl_action_ready_timeout_s_;
    }

    void update_transition_stability_gate()
    {
        if (!require_transition_stability_) {
            return;
        }
        const StabilityDiagnostics diagnostics =
            transition_stability_diagnostics();
        if (!diagnostics.gate_ok) {
            if (transition_stable_since_.has_value()) {
                RCLCPP_INFO(
                    get_logger(),
                    "Nominal transition stability reset: "
                    "height_ok=%d vz_ok=%d groundspeed_ok=%d "
                    "h_rel=%.3f eh=%.3f vz_up=%.3f Vg=%.3f dwell=%.3f",
                    diagnostics.height_ok ? 1 : 0,
                    diagnostics.vertical_speed_ok ? 1 : 0,
                    diagnostics.groundspeed_ok ? 1 : 0,
                    diagnostics.altitude_relative_m,
                    diagnostics.altitude_error_m,
                    diagnostics.vertical_speed_up_mps,
                    diagnostics.groundspeed_mps,
                    transition_stability_dwell_s());
                transition_stability_reset_count_ += 1;
            }
            transition_stable_since_.reset();
            return;
        }
        if (!transition_stable_since_.has_value()) {
            transition_stable_since_ = now();
            RCLCPP_INFO(
                get_logger(),
                "Nominal transition stability conditions acquired");
        }
    }

    double transition_stability_dwell_s() const
    {
        if (!require_transition_stability_) {
            return transition_stability_dwell_s_;
        }
        if (!transition_stable_since_.has_value()) {
            return 0.0;
        }
        return std::max(
            0.0, (now() - *transition_stable_since_).seconds());
    }

    void request_transition_mc(const bool land_after)
    {
        if (state_ == State::TransitionMc) {
            return;
        }
        if (state_ != State::HoldFw && state_ != State::TransitionFw) {
            RCLCPP_WARN(
                get_logger(),
                "TRANSITION_MC requires fixed-wing flight");
            return;
        }
        land_after_transition_ = land_after;
        set_state(State::TransitionMc, "back transition requested");
    }

    void begin_landing(const std::string &reason)
    {
        sequence_enabled_ = false;
        set_state(State::Landing, reason);
    }

    void abort_sequence(const std::string &reason)
    {
        abort_pending_ = true;
        sequence_enabled_ = false;
        last_error_ = reason;

        if (!is_armed()) {
            set_state(State::Aborted, reason);
        } else if (is_fixed_wing() ||
                   vehicle_status_.in_transition_mode) {
            land_after_transition_ = true;
            set_state(
                State::TransitionMc,
                "abort: transition to multicopter first");
        } else {
            begin_landing("abort: land");
        }
    }

    void fail_sequence(const std::string &reason)
    {
        RCLCPP_ERROR(get_logger(), "%s", reason.c_str());
        last_error_ = reason;
        terminal_error_pending_ = true;
        sequence_enabled_ = false;

        if (!is_armed()) {
            set_state(State::Error, reason);
        } else if (is_fixed_wing() ||
                   vehicle_status_.in_transition_mode) {
            land_after_transition_ = true;
            set_state(
                State::TransitionMc,
                "failure recovery: transition to multicopter");
        } else {
            begin_landing("failure recovery: land");
        }
    }

    void status_callback(
        const px4_msgs::msg::VehicleStatus::SharedPtr message)
    {
        vehicle_status_ = *message;
        status_received_ = true;

        if (vehicle_status_.failsafe &&
            state_ != State::Idle &&
            state_ != State::Completed &&
            state_ != State::Aborted &&
            state_ != State::Error &&
            !terminal_error_pending_) {
            fail_sequence("PX4 entered failsafe");
        }
    }

    void position_callback(
        const px4_msgs::msg::VehicleLocalPosition::SharedPtr message)
    {
        local_position_ = *message;
        position_received_ = true;
    }

    void ack_callback(
        const px4_msgs::msg::VehicleCommandAck::SharedPtr message)
    {
        if (!pending_command_ ||
            message->command != pending_command_->command) {
            return;
        }

        using Ack = px4_msgs::msg::VehicleCommandAck;
        if (message->result == Ack::VEHICLE_CMD_RESULT_ACCEPTED ||
            message->result == Ack::VEHICLE_CMD_RESULT_IN_PROGRESS) {
            pending_command_->accepted = true;
            RCLCPP_INFO(
                get_logger(), "PX4 accepted command %u (result=%u)",
                message->command, message->result);
            return;
        }

        if (message->result ==
            Ack::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED) {
            RCLCPP_WARN(
                get_logger(),
                "PX4 temporarily rejected command %u; retrying",
                message->command);
            pending_command_->accepted = false;
            return;
        }

        std::ostringstream stream;
        stream << "PX4 rejected command " << message->command
               << " with result " << static_cast<int>(message->result);
        pending_command_.reset();
        fail_sequence(stream.str());
    }

    void request_callback(
        const std_msgs::msg::String::SharedPtr message)
    {
        const auto command = upper_copy(message->data);
        if (command == "PREPARE") {
            begin_prepare(false);
        } else if (command == "RUN_MISSION") {
            begin_prepare(true);
        } else if (command == "TRANSITION_FW") {
            request_transition_fw();
        } else if (command == "TRANSITION_MC") {
            request_transition_mc(false);
        } else if (command == "LAND") {
            if (is_fixed_wing() ||
                vehicle_status_.in_transition_mode) {
                request_transition_mc(true);
            } else if (is_armed()) {
                begin_landing("land requested");
            }
        } else if (command == "ABORT") {
            abort_sequence("abort requested");
        } else if (command == "RESET") {
            if (!is_armed()) {
                last_error_.clear();
                set_state(State::Idle, "internal state reset");
            }
        } else {
            RCLCPP_WARN(
                get_logger(), "Unknown VTOL request: %s",
                message->data.c_str());
        }
    }

    void velocity_callback(
        const geometry_msgs::msg::Twist::SharedPtr message)
    {
        accept_velocity_command(*message, std::nullopt);
    }

    void stamped_velocity_callback(
        const geometry_msgs::msg::TwistStamped::SharedPtr message)
    {
        const auto sequence = parse_action_sequence(
            message->header.frame_id);
        if (!sequence) {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                1000,
                "Received stamped velocity setpoint without a valid seq");
        }
        accept_velocity_command(message->twist, sequence);
    }

    void a0_rl_action_callback(
        const std_msgs::msg::String::SharedPtr message)
    {
        if (message->data.empty()) {
            return;
        }
        a0_rl_action_received_at_ = now();
    }

    void accept_velocity_command(
        const geometry_msgs::msg::Twist &message,
        const std::optional<uint64_t> action_sequence)
    {
        velocity_command_ = message;
        velocity_received_at_ = now();
        velocity_received_ = true;
        // The RL environment advances Gazebo with pause + multi_step.  Since
        // this node now uses a simulation-time timer, the timer cannot fire
        // while Gazebo is paused.  Publish once immediately when a fresh
        // action arrives so the first physics step in the burst does not keep
        // using the previous action.  The regular 50 ms sim-time timer still
        // maintains Offboard heartbeat and setpoint repetition.
        if (velocity_control_is_active()) {
            publish_velocity_setpoint();
        }
        if (action_sequence) {
            publish_action_ack(*action_sequence);
        }
    }

    void publish_action_ack(const uint64_t action_sequence)
    {
        std_msgs::msg::UInt64 message{};
        message.data = action_sequence;
        action_ack_pub_->publish(message);
    }

    void start_service_callback(
        const std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr response)
    {
        const bool allowed =
            state_ == State::Idle ||
            state_ == State::Completed ||
            state_ == State::Aborted ||
            state_ == State::Error;
        response->success = allowed && !is_armed();
        response->message =
            response->success
            ? "Full VTOL mission requested"
            : "Mission cannot start in the current state";
        if (response->success) {
            begin_prepare(true);
        }
    }

    void abort_service_callback(
        const std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr response)
    {
        abort_sequence("abort service called");
        response->success = true;
        response->message = "Abort handling started";
    }

    void publish_vehicle_command(
        const uint16_t command,
        const std::array<float, 7> &parameters)
    {
        px4_msgs::msg::VehicleCommand message{};
        message.timestamp = timestamp_us();
        message.command = command;
        message.param1 = parameters[0];
        message.param2 = parameters[1];
        message.param3 = parameters[2];
        message.param4 = parameters[3];
        message.param5 = parameters[4];
        message.param6 = parameters[5];
        message.param7 = parameters[6];
        message.target_system = 1;
        message.target_component = 1;
        message.source_system = 1;
        message.source_component = 1;
        message.from_external = true;
        vehicle_command_pub_->publish(message);
    }

    bool send_command_with_retry(
        const uint16_t command,
        const std::array<float, 7> &parameters)
    {
        const auto current_time = now();
        if (!pending_command_ ||
            pending_command_->command != command) {
            pending_command_ = PendingCommand{
                command, 0, rclcpp::Time(0, 0, get_clock()->get_clock_type()),
                false};
        }

        if (pending_command_->accepted) {
            return true;
        }

        const bool never_sent = pending_command_->attempts == 0;
        const bool retry_due =
            !never_sent &&
            (current_time - pending_command_->last_sent).seconds() >=
            command_retry_s_;
        if (pending_command_->attempts >= max_command_attempts_) {
            if (retry_due) {
                std::ostringstream stream;
                stream << "command " << command
                       << " exceeded retry limit";
                fail_sequence(stream.str());
            }
            return false;
        }

        if (never_sent || retry_due) {
            publish_vehicle_command(command, parameters);
            pending_command_->last_sent = current_time;
            ++pending_command_->attempts;
            RCLCPP_INFO(
                get_logger(), "Sent PX4 command %u (attempt %d/%d)",
                command, pending_command_->attempts,
                max_command_attempts_);
        }
        return pending_command_->accepted;
    }

    void publish_position_setpoint(
        const float x, const float y, const float z,
        const float yaw)
    {
        px4_msgs::msg::OffboardControlMode mode{};
        mode.timestamp = timestamp_us();
        mode.position = true;
        offboard_mode_pub_->publish(mode);

        px4_msgs::msg::TrajectorySetpoint setpoint{};
        setpoint.timestamp = mode.timestamp;
        setpoint.position = {x, y, z};
        setpoint.velocity = {kNan, kNan, kNan};
        setpoint.acceleration = {kNan, kNan, kNan};
        setpoint.jerk = {kNan, kNan, kNan};
        setpoint.yaw = yaw;
        setpoint.yawspeed = kNan;
        trajectory_pub_->publish(setpoint);
    }

    bool action_is_fresh() const
    {
        return velocity_received_ &&
               (now() - velocity_received_at_).seconds() <=
               action_timeout_s_;
    }

    bool velocity_control_is_active() const
    {
        if (!action_is_fresh()) {
            return false;
        }
        return (
            state_ == State::HoldMc ||
            state_ == State::TransitionFw ||
            state_ == State::HoldAfterMc ||
            state_ == State::HoldFw
        );
    }

    void publish_velocity_setpoint()
    {
        px4_msgs::msg::OffboardControlMode mode{};
        mode.timestamp = timestamp_us();
        mode.position = true;
        mode.velocity = true;
        offboard_mode_pub_->publish(mode);

        double north = clamp_symmetric(
            velocity_command_.linear.x, max_horizontal_speed_mps_);
        double east = clamp_symmetric(
            velocity_command_.linear.y, max_horizontal_speed_mps_);
        const double down = clamp_symmetric(
            velocity_command_.linear.z, max_vertical_speed_mps_);
        const double yaw_rate = clamp_symmetric(
            velocity_command_.angular.z, max_yaw_rate_rps_);

        double horizontal_speed = std::hypot(north, east);
        if (horizontal_speed > max_horizontal_speed_mps_) {
            const double scale =
                max_horizontal_speed_mps_ / horizontal_speed;
            north *= scale;
            east *= scale;
            horizontal_speed = max_horizontal_speed_mps_;
        }

        if (is_fixed_wing() &&
            horizontal_speed < fw_min_forward_speed_mps_) {
            if (horizontal_speed > 1.0e-3) {
                const double scale =
                    fw_min_forward_speed_mps_ / horizontal_speed;
                north *= scale;
                east *= scale;
            } else {
                const double heading =
                    std::isfinite(local_position_.heading)
                    ? local_position_.heading : home_heading_;
                north = fw_min_forward_speed_mps_ * std::cos(heading);
                east = fw_min_forward_speed_mps_ * std::sin(heading);
            }
        }

        px4_msgs::msg::TrajectorySetpoint setpoint{};
        setpoint.timestamp = mode.timestamp;
        if (is_fixed_wing()) {
            // PX4 1.15 treats the velocity vector as a path tangent, not an
            // airspeed request.  Supplying a finite position is essential:
            // its altitude is the TECS height setpoint.  The previous NaN
            // altitude allowed uncontrolled climb/dive and extreme speed.
            const double altitude_lookahead_s = 5.0;
            const double commanded_z = std::clamp(
                static_cast<double>(local_position_.z)
                    + down * altitude_lookahead_s,
                static_cast<double>(home_z_) - 75.0,
                static_cast<double>(home_z_) - 8.0);
            setpoint.position = {
                local_position_.x,
                local_position_.y,
                static_cast<float>(commanded_z)};
        } else {
            const double altitude_lookahead_s = 2.0;
            const double commanded_z = std::clamp(
                static_cast<double>(target_z_) + down * altitude_lookahead_s,
                static_cast<double>(home_z_) - 75.0,
                static_cast<double>(home_z_) - 8.0);
            setpoint.position = {
                kNan,
                kNan,
                static_cast<float>(commanded_z)};
        }
        setpoint.velocity = {
            static_cast<float>(north),
            static_cast<float>(east),
            static_cast<float>(down)};
        setpoint.acceleration = {kNan, kNan, kNan};
        setpoint.jerk = {kNan, kNan, kNan};
        setpoint.yaw = kNan;
        setpoint.yawspeed = static_cast<float>(yaw_rate);
        trajectory_pub_->publish(setpoint);
    }

    void publish_active_setpoint()
    {
        switch (state_) {
        case State::PreflightStream:
        case State::RequestOffboard:
        case State::RequestArm:
        case State::Takeoff:
            publish_position_setpoint(
                home_x_, home_y_, target_z_, home_heading_);
            break;
        case State::HoldMc:
        case State::HoldAfterMc:
            if (velocity_control_is_active()) {
                publish_velocity_setpoint();
            } else {
                publish_position_setpoint(
                    hold_x_, hold_y_, hold_z_, hold_heading_);
            }
            break;
        case State::TransitionFw:
        case State::HoldFw:
            if (velocity_control_is_active()) {
                publish_velocity_setpoint();
            } else {
                publish_position_setpoint(
                    fw_target_x_, fw_target_y_, target_z_,
                    home_heading_);
            }
            break;
        case State::TransitionMc:
            publish_position_setpoint(
                hold_x_, hold_y_, target_z_, hold_heading_);
            break;
        default:
            break;
        }
    }

    void handle_wait_ready()
    {
        if (vehicle_is_ready()) {
            capture_home();
            set_state(
                State::PreflightStream,
                "PX4 and local position are ready");
        } else if ((now() - request_started_at_).seconds() >
                   ready_timeout_s_) {
            fail_sequence("timed out waiting for a flight-ready PX4");
        }
    }

    void handle_request_offboard()
    {
        if (offboard_is_active()) {
            set_state(State::RequestArm, "Offboard mode active");
            return;
        }
        send_command_with_retry(
            px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
            {1.0F, 6.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F});
        if (state_elapsed_s() > 10.0) {
            fail_sequence("Offboard mode activation timed out");
        }
    }

    void handle_request_arm()
    {
        if (is_armed()) {
            set_state(State::Takeoff, "vehicle armed");
            return;
        }
        send_command_with_retry(
            px4_msgs::msg::VehicleCommand::
            VEHICLE_CMD_COMPONENT_ARM_DISARM,
            {1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F});
        if (state_elapsed_s() > 10.0) {
            fail_sequence("arming timed out");
        }
    }

    void handle_takeoff()
    {
        if (!position_is_valid()) {
            fail_sequence("local position became invalid during takeoff");
            return;
        }

        const double altitude =
            static_cast<double>(home_z_ - local_position_.z);
        const double altitude_error =
            altitude - takeoff_altitude_m_;
        const bool vertical_speed_valid =
            local_position_.v_z_valid &&
            std::isfinite(local_position_.vz);
        const double vertical_speed_up_mps =
            vertical_speed_valid
            ? -static_cast<double>(local_position_.vz)
            : std::numeric_limits<double>::quiet_NaN();
        if (std::abs(altitude_error) <=
                takeoff_completion_tolerance_m_ &&
            vertical_speed_valid &&
            std::abs(vertical_speed_up_mps) <=
                takeoff_completion_vertical_speed_tolerance_mps_) {
            capture_hold_position(true);
            set_state(State::HoldMc, "takeoff altitude reached");
        } else if (state_elapsed_s() > takeoff_timeout_s_) {
            fail_sequence("takeoff timed out");
        }
    }

    void handle_transition_fw()
    {
        if (is_fixed_wing() &&
            !vehicle_status_.in_transition_mode) {
            set_state(State::HoldFw, "fixed-wing mode confirmed");
            return;
        }

        send_command_with_retry(
            px4_msgs::msg::VehicleCommand::
            VEHICLE_CMD_DO_VTOL_TRANSITION,
            {kMavVtolStateFw, 0.0F, 0.0F, 0.0F,
             0.0F, 0.0F, 0.0F});
        if (state_elapsed_s() > transition_timeout_s_) {
            fail_sequence("forward transition timed out");
        }
    }

    void handle_transition_mc()
    {
        if (is_multicopter()) {
            if (land_after_transition_) {
                begin_landing("multicopter mode recovered");
            } else {
                set_state(
                    State::HoldAfterMc,
                    "multicopter mode confirmed");
            }
            return;
        }

        send_command_with_retry(
            px4_msgs::msg::VehicleCommand::
            VEHICLE_CMD_DO_VTOL_TRANSITION,
            {kMavVtolStateMc, 0.0F, 0.0F, 0.0F,
             0.0F, 0.0F, 0.0F});
        if (state_elapsed_s() > transition_timeout_s_) {
            last_error_ = "back transition timed out";
            set_state(State::Error, last_error_);
        }
    }

    void handle_landing()
    {
        if (!is_armed()) {
            if (terminal_error_pending_) {
                set_state(State::Error, "landed after failure");
            } else if (abort_pending_) {
                set_state(State::Aborted, "abort landing complete");
            } else {
                set_state(State::Completed, "landing complete");
            }
            return;
        }

        send_command_with_retry(
            px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND,
            {0.0F, 0.0F, 0.0F, kNan,
             kNan, kNan, kNan});
        if (state_elapsed_s() > landing_timeout_s_) {
            last_error_ = "landing timed out";
            set_state(State::Error, last_error_);
        }
    }

    void timer_callback()
    {
        if (state_ != State::Landing) {
            publish_active_setpoint();
        }

        switch (state_) {
        case State::Idle:
            if (start_requested_) {
                begin_prepare(sequence_enabled_);
            }
            break;
        case State::WaitReady:
            handle_wait_ready();
            break;
        case State::PreflightStream:
            if (state_elapsed_s() >= preflight_stream_s_) {
                set_state(
                    State::RequestOffboard,
                    "Offboard heartbeat established");
            }
            break;
        case State::RequestOffboard:
            handle_request_offboard();
            break;
        case State::RequestArm:
            handle_request_arm();
            break;
        case State::Takeoff:
            handle_takeoff();
            break;
        case State::HoldMc:
            update_transition_stability_gate();
            if (sequence_enabled_ &&
                state_elapsed_s() >= hold_mc_timeout_s_) {
                fail_sequence(
                    "timed out waiting for nominal transition stability");
            } else if (
                sequence_enabled_ &&
                state_elapsed_s() >= hold_mc_s_ &&
                transition_stability_dwell_s() >=
                    transition_stability_dwell_s_) {
                request_transition_fw();
            }
            break;
        case State::TransitionFw:
            handle_transition_fw();
            break;
        case State::HoldFw:
            if (sequence_enabled_ &&
                state_elapsed_s() >= hold_fw_s_) {
                request_transition_mc(false);
            }
            break;
        case State::TransitionMc:
            handle_transition_mc();
            break;
        case State::HoldAfterMc:
            if (sequence_enabled_ &&
                state_elapsed_s() >= hold_after_mc_s_) {
                begin_landing("automatic mission complete");
            }
            break;
        case State::Landing:
            handle_landing();
            break;
        case State::Completed:
        case State::Aborted:
        case State::Error:
            start_requested_ = false;
            break;
        }

        if ((now() - last_state_publish_at_).seconds() >= 1.0) {
            publish_state();
            last_state_publish_at_ = now();
        }
    }

    rclcpp::Subscription<
        px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
    rclcpp::Subscription<
        px4_msgs::msg::VehicleLocalPosition>::SharedPtr position_sub_;
    rclcpp::Subscription<
        px4_msgs::msg::VehicleCommandAck>::SharedPtr ack_sub_;
    rclcpp::Subscription<
        std_msgs::msg::String>::SharedPtr request_sub_;
    rclcpp::Subscription<
        geometry_msgs::msg::Twist>::SharedPtr velocity_sub_;
    rclcpp::Subscription<
        geometry_msgs::msg::TwistStamped>::SharedPtr stamped_velocity_sub_;
    rclcpp::Subscription<
        std_msgs::msg::String>::SharedPtr a0_rl_action_sub_;

    rclcpp::Publisher<
        px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
    rclcpp::Publisher<
        px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
    rclcpp::Publisher<
        px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_pub_;
    rclcpp::Publisher<
        std_msgs::msg::String>::SharedPtr state_pub_;
    rclcpp::Publisher<
        std_msgs::msg::UInt64>::SharedPtr action_ack_pub_;

    rclcpp::Service<
        std_srvs::srv::Trigger>::SharedPtr start_service_;
    rclcpp::Service<
        std_srvs::srv::Trigger>::SharedPtr abort_service_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::chrono::duration<double, std::milli> command_period_{50.0};

    px4_msgs::msg::VehicleStatus vehicle_status_{};
    px4_msgs::msg::VehicleLocalPosition local_position_{};
    geometry_msgs::msg::Twist velocity_command_{};
    std::optional<PendingCommand> pending_command_;

    State state_{State::Idle};
    rclcpp::Time state_entered_at_{0, 0, RCL_ROS_TIME};
    rclcpp::Time request_started_at_{0, 0, RCL_ROS_TIME};
    rclcpp::Time velocity_received_at_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_state_publish_at_{0, 0, RCL_ROS_TIME};
    std::optional<rclcpp::Time> transition_stable_since_;
    std::optional<rclcpp::Time> a0_rl_action_received_at_;
    uint64_t transition_stability_reset_count_{0};

    bool status_received_{false};
    bool position_received_{false};
    bool velocity_received_{false};
    bool home_valid_{false};
    bool start_requested_{false};
    bool sequence_enabled_{false};
    bool land_after_transition_{false};
    bool terminal_error_pending_{false};
    bool abort_pending_{false};
    bool auto_start_mission_{false};
    bool require_transition_stability_{false};
    bool require_a0_rl_action_ready_{false};

    float home_x_{0.0F};
    float home_y_{0.0F};
    float home_z_{0.0F};
    float home_heading_{0.0F};
    float hold_x_{0.0F};
    float hold_y_{0.0F};
    float hold_z_{0.0F};
    float hold_heading_{0.0F};
    float target_z_{0.0F};
    float fw_target_x_{0.0F};
    float fw_target_y_{0.0F};

    double takeoff_altitude_m_{20.0};
    double altitude_tolerance_m_{1.0};
    double takeoff_completion_tolerance_m_{0.5};
    double takeoff_completion_vertical_speed_tolerance_mps_{0.3};
    double preflight_stream_s_{1.5};
    double hold_mc_s_{4.0};
    double transition_stability_dwell_s_{2.0};
    double transition_altitude_tolerance_m_{1.0};
    double transition_vertical_speed_tolerance_mps_{0.2};
    double transition_groundspeed_tolerance_mps_{0.2};
    double hold_mc_timeout_s_{45.0};
    double hold_fw_s_{12.0};
    double hold_after_mc_s_{3.0};
    double forward_distance_m_{60.0};
    double max_horizontal_speed_mps_{15.0};
    double max_vertical_speed_mps_{1.5};
    double max_yaw_rate_rps_{0.25};
    double fw_min_forward_speed_mps_{15.0};
    double action_timeout_s_{0.5};
    double a0_rl_action_ready_timeout_s_{0.5};
    double command_retry_s_{1.0};
    double ready_timeout_s_{45.0};
    double takeoff_timeout_s_{45.0};
    double transition_timeout_s_{25.0};
    double landing_timeout_s_{75.0};
    int max_command_attempts_{5};
    std::string last_error_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VtolCommandNode>());
    rclcpp::shutdown();
    return 0;
}
