#!/usr/bin/env python3
"""Publish zero wrench data as a workaround for Gazebo Harmonic FT sensor issue."""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Header


class FTZeroPublisher(Node):
    def __init__(self):
        super().__init__('ft_zero_publisher')
        
        # Create publisher on the FT sensor topic
        self.publisher = self.create_publisher(
            WrenchStamped,
            '/wrist_ft_sensor_zero',  # Different name to avoid conflict
            10
        )
        
        # Timer to publish at 100Hz
        self.timer = self.create_timer(0.01, self.publish_zero_wrench)
        
        self.get_logger().info('FT Zero Publisher Started')
        self.get_logger().info('Publishing zero wrench to /wrist_ft_sensor_zero at 100Hz')
        self.get_logger().warn('This is a WORKAROUND for Gazebo Harmonic FT sensor issue')
        
        self.count = 0
        
    def publish_zero_wrench(self):
        msg = WrenchStamped()
        
        # Header
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'tool0'
        
        # All zeros (no force/torque)
        msg.wrench.force.x = 0.0
        msg.wrench.force.y = 0.0
        msg.wrench.force.z = 0.0
        msg.wrench.torque.x = 0.0
        msg.wrench.torque.y = 0.0
        msg.wrench.torque.z = 0.0
        
        self.publisher.publish(msg)
        
        # Log every 100 messages
        self.count += 1
        if self.count % 100 == 0:
            self.get_logger().info(f'Published {self.count} zero wrench messages')


def main(args=None):
    rclpy.init(args=args)
    
    node = FTZeroPublisher()
    
    print("\nFT SENSOR WORKAROUND - ZERO PUBLISHER")
    print("="*60)
    print("Known issue: Gazebo Harmonic FT sensors don't publish in ROS2 Jazzy")
    print("This publishes zero wrench data for testing")
    print("Topic: /wrist_ft_sensor_zero")
    print("="*60 + "\n")
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()