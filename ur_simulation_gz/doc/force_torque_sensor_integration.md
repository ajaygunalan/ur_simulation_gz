# Force/Torque Sensor Integration in ROS2 Gazebo Simulation

**Version**: ROS2 Jazzy | Gazebo Harmonic  
**Last Updated**: January 2025

## Quick Summary

This package implements F/T sensor simulation using the Direct Bridge approach:
- **Method**: Gazebo physics → F/T plugin → gz topic → ros_gz_bridge → ROS2 topic
- **Topic**: `/wrist_ft_sensor` (geometry_msgs/msg/WrenchStamped)
- **Why**: gz_ros2_control doesn't support F/T sensors

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

## Critical Implementation Details

### Force/Torque Sensor Requirements

**1. Child Link with Mass is MANDATORY**
```xml
<!-- The F/T sensor measures forces BETWEEN parent and child -->
<joint name="ft_sensor_joint" type="fixed">
  <parent>wrist_link</parent>
  <child>tool_link</child>  <!-- THIS MUST HAVE MASS! -->
  <sensor name="ft_sensor" type="force_torque">
    <force_torque>
      <frame>child</frame>
      <measure_direction>child_to_parent</measure_direction>
    </force_torque>
  </sensor>
</joint>

<!-- Without mass, NO FORCES will be measured! -->
<link name="tool_link">
  <inertial>
    <mass value="1.0"/>  <!-- CRITICAL: Must be > 0 -->
    <inertia ixx="0.002" iyy="0.002" izz="0.001"/>
  </inertial>
</link>
```

**2. Why Child Links are Required**
- Gazebo F/T sensors measure forces BETWEEN two links
- Forces come from: gravity on mass, contacts, accelerations
- No mass = No forces to measure

### Current Implementation Status

#### What We Have (ur_simulation_gz)
```xml
<!-- In ur_gz.ros2_control.xacro -->
<xacro:ur_sensors tf_prefix="${tf_prefix}" />  <!-- Uses ur_description's sensor definition -->

<!-- In launch file -->
<node pkg="ros_gz_bridge" exec="parameter_bridge"
      args="/force_torque@geometry_msgs/msg/WrenchStamped[gz.msgs.Wrench">
  <remap from="/force_torque" to="/wrist_ft_sensor"/>
</node>
```

#### What Doesn't Work
```xml
<!-- This will FAIL with gz_ros2_control -->
<sensor name="tcp_fts_sensor">
  <state_interface name="force.x"/>
  <!-- Error: State interface 'tcp_fts_sensor/force.x' is not available -->
</sensor>
```

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

## Practical Usage

### For Simulation Only (Current Approach)
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

## Key Takeaways

1. **Current Limitation**: gz_ros2_control doesn't support F/T sensors
2. **Standard Solution**: Use Direct Bridge for simulation
3. **Physics Requirement**: Child link MUST have mass for F/T measurement
4. **Frame Convention**: Typically measure in child frame, child_to_parent direction
5. **Real Hardware**: Would use different implementation (hardware interface)

## Common Issues and Solutions

| Issue | Cause | Solution |
|-------|-------|----------|
| No F/T data published | Missing ForceTorque plugin | Add `<plugin filename="gz-sim-forcetorque-system">` |
| All values are zero | No mass on child link | Add mass to tool/payload link |
| Wrong frame_id | Incorrect sensor setup | Set proper frame in sensor definition |
| Can't use force_torque_sensor_broadcaster | gz_ros2_control limitation | Use Direct Bridge instead |

## Future Possibilities

If hardware abstraction is critical:
1. Implement custom gz_ros2_control plugin with F/T support
2. Use mock hardware for testing controllers
3. Wait for community implementation (actively discussed)

## Implementation Details

### URDF Structure
```
tool0 (UR robot end) → ft_sensor_link (0.1kg) → tool_payload (1.0kg)
```

### Key Files
- `urdf/ur_ft_sensor.xacro`: F/T sensor with child links for physics
- `urdf/ur_gz.urdf.xacro`: Includes sensor and ForceTorque plugin
- `config/ur_controllers.yaml`: No F/T broadcaster (removed)
- `launch/ur_sim_control.launch.py`: Direct Bridge configuration

### Usage Example
```python
# Subscribe to F/T data in your controller
self.ft_subscriber = self.create_subscription(
    WrenchStamped,
    '/wrist_ft_sensor',
    self.ft_callback,
    10
)

def ft_callback(self, msg):
    force_x = msg.wrench.force.x
    torque_z = msg.wrench.torque.z
    # Use F/T data for control
```

## Testing the Implementation

1. **Launch simulation**:
   ```bash
   ros2 launch ur_simulation_gz ur_sim_control.launch.py
   ```

2. **Check F/T topic**:
   ```bash
   ros2 topic echo /wrist_ft_sensor
   ```

3. **Expected output**: ~9.8N force in -Z direction (1kg tool mass × gravity)

## Conclusion

This simulation-focused implementation provides reliable F/T data using Direct Bridge. While it differs from real hardware implementation (which would use ros2_control), it's the standard and only viable approach for Gazebo simulation until gz_ros2_control adds F/T sensor support.