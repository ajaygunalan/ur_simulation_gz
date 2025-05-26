#!/usr/bin/env python3
"""Always-on Force/Torque sensor wrapper for Gazebo Sim.

Gazebo Sim FT sensors only publish when forces are detected.
This wrapper ensures continuous publishing of wrench data.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Header
import threading


class FTSensorAlwaysOn(Node):
    def __init__(self):
        super().__init__('ft_sensor_always_on')
        
        # Parameters
        self.declare_parameter('gazebo_topic', '/world/ur_world/model/ur/joint/ft_fixed_joint/sensor/tcp_fts_sensor/force_torque')
        self.declare_parameter('ros_topic', '/wrist_ft_sensor')
        self.declare_parameter('frame_id', 'tool0')
        self.declare_parameter('publish_rate', 100.0)  # Hz
        
        gazebo_topic = self.get_parameter('gazebo_topic').value
        ros_topic = self.get_parameter('ros_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        publish_rate = self.get_parameter('publish_rate').value
        
        # Current wrench data (thread-safe)
        self.current_wrench = WrenchStamped()
        self.current_wrench.header.frame_id = self.frame_id
        self.data_lock = threading.Lock()
        self.has_data = False
        
        # Subscribe to Gazebo FT sensor (when it publishes)
        self.gz_sub = self.create_subscription(
            WrenchStamped,
            gazebo_topic,
            self.gazebo_callback,
            10
        )
        
        # Publish continuously to ROS
        self.ros_pub = self.create_publisher(
            WrenchStamped,
            ros_topic,
            10
        )
        
        # Timer for continuous publishing
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_wrench)
        
        self.get_logger().info(f'Always-on FT sensor wrapper started')
        self.get_logger().info(f'Gazebo topic: {gazebo_topic}')
        self.get_logger().info(f'ROS topic: {ros_topic}')
        self.get_logger().info(f'Publishing at {publish_rate} Hz')
        
    def gazebo_callback(self, msg):
        """Update wrench data when Gazebo publishes."""
        with self.data_lock:
            self.current_wrench = msg
            self.has_data = True
            
    def publish_wrench(self):
        """Publish wrench data continuously."""
        with self.data_lock:
            # Update timestamp
            self.current_wrench.header.stamp = self.get_clock().now().to_msg()
            
            # Publish current data (zeros if no forces detected)
            self.ros_pub.publish(self.current_wrench)
            
            # Log first message
            if not self.has_data:
                self.get_logger().info('Publishing zero wrench (no forces detected yet)', once=True)


def main(args=None):
    rclpy.init(args=args)
    
    node = FTSensorAlwaysOn()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()