#!/usr/bin/env python3
"""Test FT sensor by moving robot to generate inertial forces."""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
import math


class FTSensorMotionTest(Node):
    def __init__(self):
        super().__init__('ft_sensor_motion_test')
        
        # Subscribe to FT sensor
        self.ft_sub = self.create_subscription(
            WrenchStamped,
            '/wrist_ft_sensor',
            self.ft_callback,
            10
        )
        
        # Subscribe to joint states
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )
        
        # Publisher for joint trajectory
        self.traj_pub = self.create_publisher(
            JointTrajectory,
            '/scaled_joint_trajectory_controller/joint_trajectory',
            10
        )
        
        self.joint_names = None
        self.current_positions = None
        self.ft_readings = []
        
        # Timer to send motion commands
        self.timer = self.create_timer(4.0, self.send_motion)
        self.motion_count = 0
        
        self.get_logger().info('FT Sensor Motion Test Started')
        self.get_logger().info('Will move robot to generate measurable forces/torques')
        
    def ft_callback(self, msg):
        force_mag = math.sqrt(
            msg.wrench.force.x**2 + 
            msg.wrench.force.y**2 + 
            msg.wrench.force.z**2
        )
        torque_mag = math.sqrt(
            msg.wrench.torque.x**2 + 
            msg.wrench.torque.y**2 + 
            msg.wrench.torque.z**2
        )
        
        # Log any non-zero readings
        if force_mag > 0.01 or torque_mag > 0.001:
            self.get_logger().info(
                f'FT DETECTED! Force: {force_mag:.3f}N, Torque: {torque_mag:.4f}Nm | '
                f'Fx={msg.wrench.force.x:.2f}, Fy={msg.wrench.force.y:.2f}, Fz={msg.wrench.force.z:.2f} | '
                f'Tx={msg.wrench.torque.x:.3f}, Ty={msg.wrench.torque.y:.3f}, Tz={msg.wrench.torque.z:.3f}'
            )
            self.ft_readings.append((force_mag, torque_mag))
        
    def joint_callback(self, msg):
        self.joint_names = msg.name
        self.current_positions = list(msg.position)
        
    def send_motion(self):
        if not self.joint_names or not self.current_positions:
            self.get_logger().warn('Waiting for joint states...')
            return
            
        self.motion_count += 1
        
        # Create trajectory
        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        
        # Create waypoints for a quick motion to generate forces
        point1 = JointTrajectoryPoint()
        point2 = JointTrajectoryPoint()
        
        point1.positions = self.current_positions.copy()
        point2.positions = self.current_positions.copy()
        
        # Different motions to test
        if self.motion_count % 3 == 1:
            # Quick wrist rotation
            self.get_logger().info('Test 1: Quick wrist rotation')
            if len(point2.positions) > 5:
                point2.positions[5] += 1.0  # Rotate wrist 1 rad
            point1.time_from_start = Duration(sec=0, nanosec=100000000)  # 0.1s
            point2.time_from_start = Duration(sec=0, nanosec=300000000)  # 0.3s
            
        elif self.motion_count % 3 == 2:
            # Quick elbow movement
            self.get_logger().info('Test 2: Quick elbow movement')
            if len(point2.positions) > 2:
                point2.positions[2] -= 0.5  # Move elbow
            point1.time_from_start = Duration(sec=0, nanosec=100000000)  # 0.1s
            point2.time_from_start = Duration(sec=0, nanosec=400000000)  # 0.4s
            
        else:
            # Multiple joint movement
            self.get_logger().info('Test 3: Multi-joint movement')
            if len(point2.positions) >= 6:
                point2.positions[1] += 0.3
                point2.positions[2] -= 0.3
                point2.positions[4] += 0.5
            point1.time_from_start = Duration(sec=0, nanosec=100000000)  # 0.1s
            point2.time_from_start = Duration(sec=0, nanosec=500000000)  # 0.5s
        
        traj.points = [point1, point2]
        
        self.get_logger().info('Sending rapid motion command...')
        self.traj_pub.publish(traj)
        
        # Update current position for next motion
        self.current_positions = list(point2.positions)


def main(args=None):
    rclpy.init(args=args)
    node = FTSensorMotionTest()
    
    print("\n" + "="*60)
    print("FORCE/TORQUE SENSOR - MOTION TEST")
    print("="*60)
    print("\nThis test will:")
    print("1. Send rapid motion commands to the robot")
    print("2. Monitor FT sensor for inertial forces/torques")
    print("3. Display any detected forces during motion")
    print("\nNOTE: FT sensors measure reaction forces!")
    print("Static robots show zero force (gravity compensated)")
    print("="*60 + "\n")
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        print(f"\nTest complete. Detected {len(node.ft_readings)} non-zero readings")
        if node.ft_readings:
            max_force = max(f for f, _ in node.ft_readings)
            max_torque = max(t for _, t in node.ft_readings)
            print(f"Max force: {max_force:.3f}N, Max torque: {max_torque:.4f}Nm")
        
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()