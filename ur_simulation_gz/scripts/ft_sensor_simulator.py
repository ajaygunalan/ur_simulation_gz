#!/usr/bin/env python3
"""Simulate FT sensor data for testing when Gazebo sensor isn't working."""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Header
import math
import time


class FTSensorSimulator(Node):
    def __init__(self):
        super().__init__('ft_sensor_simulator')
        
        # Create publisher on the same topic
        self.publisher = self.create_publisher(
            WrenchStamped,
            '/wrist_ft_sensor_sim',  # Different topic to not conflict
            10
        )
        
        # Timer to publish at 100Hz
        self.timer = self.create_timer(0.01, self.publish_wrench)
        
        self.time_start = time.time()
        self.get_logger().info('FT Sensor Simulator Started')
        self.get_logger().info('Publishing simulated wrench data to /wrist_ft_sensor_sim')
        
    def publish_wrench(self):
        msg = WrenchStamped()
        
        # Header
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'tool0'  # Or 'ft_sensor_link'
        
        # Simulate some varying forces/torques
        t = time.time() - self.time_start
        
        # Base values (gravity on tool)
        msg.wrench.force.x = 0.0
        msg.wrench.force.y = 0.0
        msg.wrench.force.z = -9.81 * 0.5  # 0.5kg tool weight
        
        # Add some sinusoidal variations to simulate motion
        msg.wrench.force.x += 2.0 * math.sin(0.5 * t)
        msg.wrench.force.y += 1.5 * math.cos(0.7 * t)
        msg.wrench.force.z += 0.5 * math.sin(1.0 * t)
        
        # Torques
        msg.wrench.torque.x = 0.1 * math.sin(0.3 * t)
        msg.wrench.torque.y = 0.15 * math.cos(0.4 * t)
        msg.wrench.torque.z = 0.05 * math.sin(0.6 * t)
        
        self.publisher.publish(msg)
        
        # Log every 100th message
        if int(t * 100) % 100 == 0:
            self.get_logger().info(
                f'Published: Fx={msg.wrench.force.x:.2f}, Fy={msg.wrench.force.y:.2f}, '
                f'Fz={msg.wrench.force.z:.2f} N, Tx={msg.wrench.torque.x:.3f}, '
                f'Ty={msg.wrench.torque.y:.3f}, Tz={msg.wrench.torque.z:.3f} Nm'
            )


def main(args=None):
    rclpy.init(args=args)
    
    node = FTSensorSimulator()
    
    print("\nFT SENSOR SIMULATOR")
    print("="*50)
    print("This simulates FT sensor data for testing.")
    print("Publishing to: /wrist_ft_sensor_sim")
    print("="*50 + "\n")
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()