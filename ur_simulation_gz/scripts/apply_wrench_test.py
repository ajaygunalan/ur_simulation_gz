#!/usr/bin/env python3
"""Apply wrench to robot end-effector to test FT sensor."""

import rclpy
from rclpy.node import Node
from ros_gz_interfaces.srv import ControlWorld
from geometry_msgs.msg import WrenchStamped
import time


class ApplyWrenchTest(Node):
    def __init__(self):
        super().__init__('apply_wrench_test')
        
        # Subscribe to FT sensor
        self.ft_sub = self.create_subscription(
            WrenchStamped,
            '/wrist_ft_sensor',
            self.ft_callback,
            10
        )
        
        # Service client for world control
        self.world_control_client = self.create_client(
            ControlWorld,
            '/world/empty/control'
        )
        
        self.get_logger().info('Waiting for world control service...')
        while not self.world_control_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
            
        self.get_logger().info('Apply Wrench Test Started')
        
        # Timer to apply forces periodically
        self.timer = self.create_timer(3.0, self.apply_wrench)
        self.force_applied = False
        
    def ft_callback(self, msg):
        # Only log non-zero forces
        force_mag = (msg.wrench.force.x**2 + msg.wrench.force.y**2 + msg.wrench.force.z**2)**0.5
        torque_mag = (msg.wrench.torque.x**2 + msg.wrench.torque.y**2 + msg.wrench.torque.z**2)**0.5
        
        if force_mag > 0.1 or torque_mag > 0.01:
            self.get_logger().info(
                f'FT Sensor: Force={force_mag:.2f}N, Torque={torque_mag:.3f}Nm | '
                f'F=[{msg.wrench.force.x:.1f}, {msg.wrench.force.y:.1f}, {msg.wrench.force.z:.1f}] '
                f'T=[{msg.wrench.torque.x:.2f}, {msg.wrench.torque.y:.2f}, {msg.wrench.torque.z:.2f}]'
            )
            
    def apply_wrench(self):
        """Apply a wrench to the robot tool."""
        if not self.force_applied:
            self.get_logger().info('Applying test forces to robot end-effector...')
            self.get_logger().warn('Note: This is a placeholder - actual force application requires Gazebo services')
            self.force_applied = True
            
            # In reality, we'd need to use Gazebo's apply_link_wrench service
            # For now, just log that we would apply forces
            self.get_logger().info('To see FT sensor data:')
            self.get_logger().info('1. Move the robot using joint commands')
            self.get_logger().info('2. Or place objects on the end-effector')
            self.get_logger().info('3. Or enable gravity compensation')


def main(args=None):
    rclpy.init(args=args)
    node = ApplyWrenchTest()
    
    print("\nFORCE/TORQUE SENSOR - WRENCH APPLICATION TEST")
    print("="*50)
    print("This test monitors the FT sensor while forces are applied.")
    print("Note: FT sensors only show readings when forces are present!")
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