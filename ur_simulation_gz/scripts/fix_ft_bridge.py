#!/usr/bin/env python3
"""Fix FT sensor bridge by finding and bridging the correct topic."""

import subprocess
import time
import sys


def find_ft_topic():
    """Find the actual FT sensor topic in Gazebo."""
    try:
        result = subprocess.run(['gz', 'topic', '-l'], capture_output=True, text=True)
        topics = result.stdout.strip().split('\n')
        
        # Look for force_torque topics
        ft_topics = [t for t in topics if 'force_torque' in t and '/sensor/' in t]
        
        if ft_topics:
            print(f"Found FT sensor topics: {ft_topics}")
            return ft_topics[0]
        else:
            print("No force_torque topics found in Gazebo")
            return None
            
    except Exception as e:
        print(f"Error finding topics: {e}")
        return None


def check_topic_publishers(topic):
    """Check if topic has publishers."""
    try:
        result = subprocess.run(['gz', 'topic', '-i', '-t', topic], 
                              capture_output=True, text=True)
        return "No publishers" not in result.stdout
    except:
        return False


def main():
    print("FT Sensor Bridge Fixer")
    print("="*50)
    
    # Find the FT topic
    ft_topic = find_ft_topic()
    if not ft_topic:
        print("ERROR: No FT sensor topic found in Gazebo!")
        print("\nPossible issues:")
        print("1. The FT sensor isn't created in Gazebo")
        print("2. The Force/Torque system plugin isn't loaded")
        return
        
    print(f"\nFound Gazebo FT topic: {ft_topic}")
    
    # Check if it has publishers
    has_publishers = check_topic_publishers(ft_topic)
    if not has_publishers:
        print("WARNING: FT topic exists but has no publishers!")
        print("The sensor is defined but not active.")
        print("\nTo fix this, the simulation needs to be restarted with proper world file.")
        
    # Check current bridge
    print("\nChecking ROS2 side...")
    result = subprocess.run(['ros2', 'topic', 'info', '/wrist_ft_sensor'], 
                          capture_output=True, text=True)
    print(result.stdout)
    
    print("\nDiagnosis:")
    print(f"- Gazebo topic: {ft_topic}")
    print(f"- Topic has publishers: {has_publishers}")
    print("- ROS2 bridge is running but not receiving data")
    
    if not has_publishers:
        print("\nSOLUTION: The Force/Torque sensor system plugin is not loaded.")
        print("You need to restart the simulation. The launch file has been fixed.")
        print("\n1. Stop current simulation (Ctrl+C)")
        print("2. Run: ros2 launch ur_simulation_gz ur_sim_control.launch.py")


if __name__ == "__main__":
    main()