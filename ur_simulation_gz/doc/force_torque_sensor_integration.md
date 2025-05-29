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

**Why Child Links are Required:**
- Gazebo F/T sensors measure forces BETWEEN two links
- Forces come from: gravity on mass, contacts, accelerations
- No mass = No forces to measure

## Architecture Comparison

### Joint States - Two Approaches Available

#### 1. ros2_control Architecture (Recommended)
```
Gazebo Physics → gz_ros2_control → Hardware Interface → joint_state_broadcaster → /joint_states
```
- ✅ Same code for simulation and real hardware
- ✅ Synchronized control loops
- ✅ Full hardware abstraction

#### 2. Direct Bridge (Quick setup)
```
Gazebo Physics → Joint State Plugin → gz topic → ros_gz_bridge → /joint_states
```
- ✅ Simple setup
- ❌ Different from real hardware implementation

### Force/Torque Sensors - Only One Approach Works

#### Direct Bridge (Currently the ONLY option)
```
Gazebo Physics → ForceTorque Plugin → gz topic → ros_gz_bridge → /wrist_ft_sensor
```
- ✅ Works reliably with Gazebo physics
- ❌ No hardware abstraction (different from real robot)
- ❌ gz_ros2_control does NOT support F/T sensors

## The gz_ros2_control Limitation

**Evidence from source code (gz_system.cpp):**
```cpp
void GazeboSimSystem::registerSensors(...) {
  // Only searches for IMU components
  this->dataPtr->ecm->Each<sim::components::Imu, ...>
  // NO implementation for ForceTorque components!
}
```

**What this means:**
- Joint interfaces: ✅ Fully supported
- IMU sensors: ✅ Supported
- F/T sensors: ❌ NOT implemented

## Current Implementation Status

### What We Have (ur_simulation_gz)
```xml
<!-- In ur_ft_sensor.xacro -->
<joint name="${prefix}ft_sensor_joint" type="revolute">
  <!-- Sensor configuration -->
</joint>

<!-- In launch file -->
<node pkg="ros_gz_bridge" exec="parameter_bridge"
      args="/force_torque@geometry_msgs/msg/WrenchStamped[gz.msgs.Wrench">
  <remap from="/force_torque" to="/wrist_ft_sensor"/>
</node>
```

### What Doesn't Work
```xml
<!-- This will FAIL with gz_ros2_control -->
<sensor name="tcp_fts_sensor">
  <state_interface name="force.x"/>
  <!-- Error: State interface 'tcp_fts_sensor/force.x' is not available -->
</sensor>
```

## Implementation Details

### URDF Structure
```
base_link → ... → tool0 → ft_sensor_link → tool_payload
                    ↑            ↑
                 fixed      revolute (with F/T sensor)
```

### Key Files
- `urdf/ur_ft_sensor.xacro`: F/T sensor with proper joint structure
- `urdf/ur_gz.ros2_control.xacro`: Includes joint state interface for transform publishing
- `config/ur_controllers.yaml`: No F/T broadcaster (removed)
- `launch/ur_sim_control.launch.py`: Direct Bridge configuration

### Common Issues and Solutions

| Issue | Cause | Solution |
|-------|-------|----------|
| No transform to tool_payload | Revolute joint not published | Add joint to ros2_control config |
| No F/T data | Fixed joint used | Change to revolute with zero limits |
| All values zero | No mass on child link | Add mass to payload |
| Transform errors | Using two revolute joints | Use fixed + revolute combo |
| Wrong frame_id | Incorrect sensor setup | Set proper frame in sensor definition |
| Can't use force_torque_sensor_broadcaster | gz_ros2_control limitation | Use Direct Bridge instead |

## Practical Usage

### For Simulation (Current Approach)
```python
# In your launch file
ft_bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=['/force_torque@geometry_msgs/msg/WrenchStamped[gz.msgs.Wrench'],
    remappings=[('/force_torque', '/wrist_ft_sensor')]
)

# Subscribe in your controller
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

### For Real Hardware (Future)
```xml
<!-- Real UR robot with ur_robot_driver -->
<ros2_control name="ur" type="system">
  <hardware>
    <plugin>ur_robot_driver/URPositionHardwareInterface</plugin>
  </hardware>
  <!-- F/T data comes through hardware interface -->
</ros2_control>
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
   ```
   Expected output: ~9.8N force in -Z direction (1kg tool mass × gravity)

## Key Takeaways

1. **Gazebo F/T sensors require movable joints** - Physics engine needs dynamic constraints
2. **Transform publishing is critical** - Revolute joints need explicit state publishing
3. **Child link MUST have mass** - No mass = no forces to measure
4. **Fixed joints break F/T sensors** - Fundamental Gazebo limitation
5. **Real Hardware uses different implementation** - Hardware interface instead of Direct Bridge

## Future Possibilities

If hardware abstraction is critical:
1. Implement custom gz_ros2_control plugin with F/T support
2. Use mock hardware for testing controllers
3. Wait for community implementation (actively discussed)

Until then, Direct Bridge remains the standard approach for F/T simulation in Gazebo.