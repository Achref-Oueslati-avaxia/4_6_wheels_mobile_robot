# My Diff Drive Setup Checklist (ROS2, 4/6 Wheels)

## 1. Add/Change in Project
- [ ] Add wheel links/joints in `mobile_base.xacro` (4 or 6 wheels)
- [ ] Add wheel joints to `mobile_base.ros2_control.xacro`
- [ ] Reference all Xacros in main URDF
- [ ] Add all wheel joints to YAML: `left_wheel_names`, `right_wheel_names`
- [ ] Set `open_loop: true` in controller YAML
- [ ] Launch robot, controllers, and RViz in launch.py
- [ ] Update config and launch files in `src/my_robot_bringup` as needed

## 2. Build & Source
```bash
cd /home/achref/ROS2_Humble/4_6_wheels_mobile_robot
colcon build --symlink-install
source install/setup.bash
```

## 3. Run Everything
```bash
ros2 launch my_robot_bringup my_robot.launch.py
```

## 4. Movement Commands
```bash
# Teleop (stamped)
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/diff_drive_controller/cmd_vel -p stamped:=true

# Direct movement (stamped)
ros2 topic pub /diff_drive_controller/cmd_vel geometry_msgs/msg/TwistStamped '{twist: {linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}'  # Forward
ros2 topic pub /diff_drive_controller/cmd_vel geometry_msgs/msg/TwistStamped '{twist: {linear: {x: -0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}' # Backward
ros2 topic pub /diff_drive_controller/cmd_vel geometry_msgs/msg/TwistStamped '{twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}}'   # Turn left
ros2 topic pub /diff_drive_controller/cmd_vel geometry_msgs/msg/TwistStamped '{twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.5}}}'  # Turn right
ros2 topic pub /diff_drive_controller/cmd_vel geometry_msgs/msg/TwistStamped '{twist: {linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}}'   # Forward + turn left
```

## 5. System Introspection & Diagnostics
```bash
# Controllers
ros2 control list_controllers
ros2 control list_hardware_interfaces
ros2 control list_hardware_components

# Topics
ros2 topic list
ros2 topic info /diff_drive_controller/cmd_vel
ros2 topic info /joint_states
ros2 topic echo /joint_states
ros2 topic echo /diff_drive_controller/odom

# Nodes
ros2 node list
ros2 node info /controller_manager

# Params
ros2 param list
ros2 param get /controller_manager
```

## 6. For 6 Wheels
- Add extra wheel links/joints in Xacro
- [ ] Add new joints to YAML

## 7. Quick Reminders
- [ ] Swap mock hardware for real hardware plugins if needed
- [ ] Check config files in `src/my_robot_bringup/config` for wheel and controller settings
- [ ] Use RViz for visualization and debugging
