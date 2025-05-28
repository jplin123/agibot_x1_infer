#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import csv
import datetime

class FullActuatorLogger(Node):
    def __init__(self):
        super().__init__('full_actuator_logger')
        self.subscription = self.create_subscription(
            JointState,
            '/actuator_states',  # Topic published by DCU
            self.listener_callback,
            10)

        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.filename = f"./log/all_actuator_states_{timestamp}.csv"
        self.file = open(self.filename, mode='w', newline='')
        self.csv_writer = csv.writer(self.file)

        # Write header
        self.csv_writer.writerow(["timestamp", "name", "effort", "velocity", "position"])

    def listener_callback(self, msg: JointState):
        now = self.get_clock().now().to_msg()
        timestamp = f"{now.sec}.{now.nanosec:09d}"

        for name, effort, velocity, position in zip(
            msg.name, msg.effort, msg.velocity, msg.position
        ):
            self.csv_writer.writerow([timestamp, name, effort, velocity, position])

    def destroy_node(self):
        self.file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = FullActuatorLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down logger...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
