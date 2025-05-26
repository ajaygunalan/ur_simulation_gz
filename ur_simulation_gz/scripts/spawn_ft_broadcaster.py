#!/usr/bin/env python3
"""Spawn force torque sensor broadcaster as a workaround."""

import subprocess
import time
import sys


def main():
    print("Attempting to spawn Force Torque Sensor Broadcaster...")
    print("="*50)
    
    # Check if controller manager is available
    try:
        result = subprocess.run(['ros2', 'control', 'list_controllers'], 
                              capture_output=True, text=True, timeout=2)
        print("Current controllers:")
        print(result.stdout)
    except:
        print("Controller manager not available")
        return
    
    # Try to spawn force_torque_sensor_broadcaster
    print("\nSpawning force_torque_sensor_broadcaster...")
    try:
        result = subprocess.run([
            'ros2', 'control', 'load_controller', 
            '--set-state', 'active',
            'force_torque_sensor_broadcaster'
        ], capture_output=True, text=True)
        
        print(result.stdout)
        if result.stderr:
            print("Error:", result.stderr)
            
    except Exception as e:
        print(f"Failed to spawn broadcaster: {e}")
    
    # Check if it worked
    time.sleep(1)
    print("\nChecking /wrist_ft_sensor topic...")
    result = subprocess.run(['ros2', 'topic', 'hz', '/wrist_ft_sensor', '--window', '10'], 
                          capture_output=True, text=True, timeout=3)
    print(result.stdout)


if __name__ == "__main__":
    main()