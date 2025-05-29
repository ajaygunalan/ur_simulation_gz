# Force/Torque Sensor Integration in ROS2 Gazebo Simulation

**Version**: ROS2 Jazzy | Gazebo Harmonic  
**Last Updated**: January 2025

## Quick Summary

This package implements F/T sensor simulation using the Direct Bridge approach:
- **Method**: Gazebo physics → F/T plugin → gz topic → ros_gz_bridge → ROS2 topic
- **Topic**: `/wrist_ft_sensor` (geometry_msgs/msg/WrenchStamped)
- **Why**: gz_ros2_control doesn't support F/T sensors

## Critical Implementation Requirements

### 1. Joint Type MUST be Revolute or Prismatic

**❌ WRONG - Fixed joints DO NOT work:**
```xml
<joint name="ft_sensor_joint" type="fixed">  <!-- NO FORCE MEASUREMENT! -->
  <parent link="ft_sensor_link"/>
  <child link="tool_payload"/>
</joint>
```

**✅ CORRECT - Use revolute with zero limits:**
```xml
<joint name="ft_sensor_joint" type="revolute">
  <parent link="ft_sensor_link"/>
  <child link="tool_payload"/>
  <axis xyz="0 0 1"/>
  <limit lower="0" upper="0" effort="1000" velocity="0"/>
  <dynamics damping="100.0" friction="100.0"/>  <!-- Acts as fixed -->
</joint>
```

### 2. Joint State MUST be Published for Transforms

Add to ros2_control configuration:
```xml
<!-- In ur_gz.ros2_control.xacro -->
<joint name="${tf_prefix}ft_sensor_joint">
  <state_interface name="position">
    <param name="initial_value">0.0</param>
  </state_interface>
  <state_interface name="velocity"/>
</joint>
```

### 3. Child Link MUST Have Mass

```xml
<link name="tool_payload">
  <inertial>
    <mass value="1.0"/>  <!-- CRITICAL: Must be > 0 -->
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <inertia ixx="0.002" iyy="0.002" izz="0.001"/>
  </inertial>
</link>
```

## Architecture Overview

### Current Implementation (Direct Bridge)
```
Gazebo Physics → ForceTorque Plugin → gz topic → ros_gz_bridge → /wrist_ft_sensor
```

### Why Not ros2_control?
gz_ros2_control does NOT support F/T sensor interfaces. The source code only implements:
- ✅ Joint interfaces (position, velocity, effort)
- ✅ IMU sensors
- ❌ Force/Torque sensors

## Implementation Details

### URDF Structure
```
base_link → ... → tool0 → ft_sensor_link → tool_payload
                    ↑            ↑
                 fixed      revolute (with F/T sensor)
```

### Key Components

1. **ur_ft_sensor.xacro**: Defines F/T sensor with proper joint structure
2. **ur_gz.ros2_control.xacro**: Includes joint state interface for transform publishing
3. **Launch file**: Configures ros_gz_bridge for F/T data

### Common Issues and Solutions

| Issue | Cause | Solution |
|-------|-------|----------|
| No transform to tool_payload | Revolute joint not published | Add joint to ros2_control config |
| No F/T data | Fixed joint used | Change to revolute with zero limits |
| All values zero | No mass on child link | Add mass to payload |
| Transform errors | Using two revolute joints | Use fixed + revolute combo |

## Usage Example

```python
# Subscribe to F/T data
self.ft_subscriber = self.create_subscription(
    WrenchStamped,
    '/wrist_ft_sensor',
    self.ft_callback,
    10
)

def ft_callback(self, msg):
    force = [msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z]
    torque = [msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z]
    # Use for admittance control, collision detection, etc.
```

## Testing

1. **Launch simulation**:
   ```bash
   ros2 launch ur_simulation_gz ur_sim_control.launch.py
   ```

2. **Verify transforms**:
   ```bash
   ros2 run tf2_tools view_frames
   # Should show: base_link → ... → tool0 → ft_sensor_link → tool_payload
   ```

3. **Check F/T data**:
   ```bash
   ros2 topic echo /wrist_ft_sensor
   # Expect ~9.8N in -Z (gravity on 1kg payload)
   ```

## Key Lessons Learned

1. **Gazebo F/T sensors require movable joints** - Physics engine needs dynamic constraints
2. **Transform publishing is critical** - Revolute joints need explicit state publishing
3. **Mass is mandatory** - No mass = no forces to measure
4. **Fixed joints break F/T sensors** - Fundamental Gazebo limitation

## Future Improvements

When gz_ros2_control adds F/T support:
- Switch to hardware-abstracted implementation
- Use force_torque_sensor_broadcaster
- Unified interface for sim and real hardware

Until then, Direct Bridge remains the standard approach for F/T simulation in Gazebo.