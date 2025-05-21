#include "dcu_driver_module/current_protector_publisher.h"

CurrentProtectorPublisher::CurrentProtectorPublisher(rclcpp::Node::SharedPtr node)
    : node_(node),
      protector_(5.0f, 0.9f, 500) {
    pub_ = node_->create_publisher<my_ros2_proto::msg::ActuatorStatus>("/actuator_status", 10);
}

void CurrentProtectorPublisher::Update(const std::string& name, float current) {
    bool disabled = protector_.Update(name, current);

    my_ros2_proto::msg::ActuatorStatus msg;
    msg.name = name;
    msg.is_disabled = disabled;
    msg.current = current;

    pub_->publish(msg);
}