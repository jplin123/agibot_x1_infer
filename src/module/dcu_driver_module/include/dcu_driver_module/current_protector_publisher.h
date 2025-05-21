#pragma once
#include "rclcpp/rclcpp.hpp"
#include "my_ros2_proto/msg/actuator_status.hpp"
#include "dcu_driver_module/current_protector.h"

class CurrentProtectorPublisher {
public:
    CurrentProtectorPublisher(rclcpp::Node::SharedPtr node);
    void Update(const std::string& name, float current);
private:
    CurrentProtector protector_;
    rclcpp::Publisher<my_ros2_proto::msg::ActuatorStatus>::SharedPtr pub_;
    rclcpp::Node::SharedPtr node_;
};