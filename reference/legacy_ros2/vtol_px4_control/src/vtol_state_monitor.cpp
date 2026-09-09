// Copyright 2026 Weicheng

#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

using namespace std::chrono_literals;

class VtolStateMonitor : public rclcpp::Node
{
public:
    VtolStateMonitor()
    : Node("vtol_state_monitor")
    {
        // PX4 /fmu/out topics use best-effort QoS.
        const auto qos = rclcpp::SensorDataQoS().keep_last(5);

        status_sub_ =
            create_subscription<px4_msgs::msg::VehicleStatus>(
                "/fmu/out/vehicle_status",
                qos,
                [this](const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
                    vehicle_status_ = *msg;
                    status_received_ = true;
                    report_state_change();
                });

        position_sub_ =
            create_subscription<px4_msgs::msg::VehicleLocalPosition>(
                "/fmu/out/vehicle_local_position",
                qos,
                [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
                    local_position_ = *msg;
                    position_received_ = true;
                });

        report_timer_ =
            rclcpp::create_timer(
                this,
                get_clock(),
                rclcpp::Duration(1s),
                std::bind(
                    &VtolStateMonitor::report_periodic_status,
                    this));

        RCLCPP_INFO(
            get_logger(),
            "VTOL state monitor started. Waiting for PX4 topics...");
    }

private:
    static std::string vehicle_type_name(const uint8_t vehicle_type)
    {
        switch (vehicle_type) {
        case 1:
            return "MULTICOPTER";

        case 2:
            return "FIXED_WING";

        default:
            return "UNKNOWN(" + std::to_string(vehicle_type) + ")";
        }
    }

    static std::string arming_state_name(const uint8_t state)
    {
        switch (state) {
        case 1:
            return "DISARMED";

        case 2:
            return "ARMED";

        default:
            return "STATE(" + std::to_string(state) + ")";
        }
    }

    void report_state_change()
    {
        const auto &status = vehicle_status_;

        const bool changed =
            !previous_state_valid_ ||
            status.vehicle_type != previous_vehicle_type_ ||
            status.in_transition_mode != previous_in_transition_mode_ ||
            status.in_transition_to_fw != previous_in_transition_to_fw_ ||
            status.failsafe != previous_failsafe_ ||
            status.arming_state != previous_arming_state_;

        if (!changed) {
            return;
        }

        RCLCPP_INFO(
            get_logger(),
            "STATE | armed=%s | type=%s | transition=%s | "
            "to_fixed_wing=%s | failsafe=%s | preflight=%s",
            arming_state_name(status.arming_state).c_str(),
            vehicle_type_name(status.vehicle_type).c_str(),
            status.in_transition_mode ? "true" : "false",
            status.in_transition_to_fw ? "true" : "false",
            status.failsafe ? "true" : "false",
            status.pre_flight_checks_pass ? "pass" : "fail");

        previous_vehicle_type_ = status.vehicle_type;
        previous_in_transition_mode_ =
            status.in_transition_mode;
        previous_in_transition_to_fw_ =
            status.in_transition_to_fw;
        previous_failsafe_ = status.failsafe;
        previous_arming_state_ = status.arming_state;
        previous_state_valid_ = true;
    }

    void report_periodic_status()
    {
        if (!status_received_) {
            RCLCPP_WARN(
                get_logger(),
                "No /fmu/out/vehicle_status data received.");
            return;
        }

        if (!position_received_) {
            RCLCPP_WARN(
                get_logger(),
                "VehicleStatus received, but no local-position data.");
            return;
        }

        const auto &position = local_position_;

        const float horizontal_speed =
            std::sqrt(
                position.vx * position.vx +
                position.vy * position.vy);

        // PX4 local position uses NED coordinates:
        // x = north, y = east, z = down.
        // Therefore altitude above the origin is -z.
        const float altitude = -position.z;

        RCLCPP_INFO(
            get_logger(),
            "POSITION | north=%.2f m | east=%.2f m | "
            "altitude=%.2f m | horizontal_speed=%.2f m/s | "
            "vertical_down_speed=%.2f m/s | nav_state=%u",
            static_cast<double>(position.x),
            static_cast<double>(position.y),
            static_cast<double>(altitude),
            static_cast<double>(horizontal_speed),
            static_cast<double>(position.vz),
            static_cast<unsigned int>(
                vehicle_status_.nav_state));
    }

    rclcpp::Subscription<
        px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;

    rclcpp::Subscription<
        px4_msgs::msg::VehicleLocalPosition>::SharedPtr position_sub_;

    rclcpp::TimerBase::SharedPtr report_timer_;

    px4_msgs::msg::VehicleStatus vehicle_status_{};
    px4_msgs::msg::VehicleLocalPosition local_position_{};

    bool status_received_{false};
    bool position_received_{false};
    bool previous_state_valid_{false};

    uint8_t previous_vehicle_type_{
        std::numeric_limits<uint8_t>::max()};

    uint8_t previous_arming_state_{
        std::numeric_limits<uint8_t>::max()};

    bool previous_in_transition_mode_{false};
    bool previous_in_transition_to_fw_{false};
    bool previous_failsafe_{false};
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    try {
        rclcpp::spin(
            std::make_shared<VtolStateMonitor>());
    } catch (const std::exception &exception) {
        std::cerr
            << "vtol_state_monitor exception: "
            << exception.what()
            << std::endl;

        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
