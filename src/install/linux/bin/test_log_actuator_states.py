#!/usr/bin/env python3

import os
import time
import subprocess
import threading
from datetime import datetime
from rclpy.node import Node
import rclpy
from sensor_msgs.msg import JointState

class ActuatorStatePublisher(Node):
    def __init__(self):
        super().__init__('test_actuator_state_pub')
        self.publisher_ = self.create_publisher(JointState, '/actuator_states', 10)
        self.timer_ = self.create_timer(0.1, self.publish_dummy_data)

    def publish_dummy_data(self):
        msg = JointState()
        msg.name = ['motor_1', 'motor_2']
        msg.effort = [1.2, 2.5]
        msg.velocity = [0.1, 0.2]
        msg.position = [0.05, 0.1]
        self.publisher_.publish(msg)
        self.get_logger().info("Published dummy actuator state")

def run_logger():
    """Run the logger script as a subprocess."""
    subprocess.run(['python3', 'install/linux/bin/log_all_actuator_states.py'], check=True)

def main():
    # Start logger in a separate thread
    logger_thread = threading.Thread(target=run_logger, daemon=True)
    logger_thread.start()

    # Allow logger to start
    time.sleep(1)

    # Start ROS2 node and publish test data
    rclpy.init()
    node = ActuatorStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

    # Wait a bit to ensure logger flushes
    time.sleep(2)

    # Check output directory
    log_dir = "./log/actuator_state"
    assert os.path.exists(log_dir), f"Log directory {log_dir} does not exist"

    # Find recent log file
    files = sorted([f for f in os.listdir(log_dir) if f.endswith(".csv")], reverse=True)
    assert files, "No CSV log file found"

    log_path = os.path.join(log_dir, files[0])
    print(f"✅ Found log: {log_path}")

    with open(log_path, 'r') as f:
        lines = f.readlines()
        assert len(lines) > 1, "Log file has no data rows"
        print(f"✅ Log contains {len(lines)-1} data entries")

if __name__ == '__main__':
    main()
