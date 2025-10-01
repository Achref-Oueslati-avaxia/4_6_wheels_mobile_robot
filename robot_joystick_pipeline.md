# Robot Joystick Control Pipeline

## 1. Web Joystick

- The joystick is a widget on our web interface at `http://avaxia-robotics.local:3000/`
- When you move the joystick, the web app sends a command as a JSON message using MQTT
- Example message:

  ```json
  {"direction": 8, "speed": 50}
  ```

## 2. MQTT Broker and Bridge

- The web app sends the message to the MQTT broker at `avaxia-robotics.local:1888`
- The topic used is `robot/control`
- The bridge subscribes to `robot/control` and publishes the same message to the ROS 2 topic `/robot/joystick_cmd`

  ```yaml
    mqtt_topics:
        - robot/control
    robot/control:
        ros_topic: /robot/joystick_cmd
  ```

- This mapping is set in the YAML config file. The bridge acts as a translator between MQTT and ROS 2.

## 3. ROS 2 Node and Conversion

- The node `joystick_mqtt_to_twist.py`
subscribes to `/robot/joystick_cmd`
- It receives the JSON message and parses it
- The node converts the message to a velocity command (`TwistStamped`)
- The node reads `direction` and `speed`, multiplies speed by a scale, and sets velocity fields in a `TwistStamped` message
- If direction is 8, it sets `twist.twist.linear.x` to the scaled speed and `twist.twist.angular.z` to zero, then publishes the message to `/diff_drive_controller/cmd_vel`

### How the conversion works

- The code parses the JSON message:

  ```python
  # Parse the incoming JSON string into a Python dictionary
  data = json.loads(msg.data)
  # Get the speed value, convert to float, and apply scale
  s = float(data.get('speed', 0)) * SCALE
  # Get the direction value, convert to int, default to 5
  direction = int(data.get('direction', 5))
  # Create a new TwistStamped message
  twist = TwistStamped()
  # If direction is 8 (forward)
  if direction == 8:
      # Set forward velocity
      twist.twist.linear.x = s
      # Set turning velocity to zero
      twist.twist.angular.z = 0.0
  # ... other directions handled similarly ...
  # Publish the velocity command
  self.pub.publish(twist)
  ```

- For example, if direction is 8 and speed is 50:
  - `linear.x = 0.01 * 50 = 0.5`
  - `angular.z = 0.0`
- This means the robot moves forward in RViz for 0.5 meters per second

## 4. Publishing to Controller

- The node publishes the velocity command to `/diff_drive_controller/cmd_vel`

## 5. Controller Manager

- The controller manager subscribes to /diff_drive_controller/cmd_vel, receives the command, and sends it to the mock hardware interface.

## 6. Robot Movement in RViz

- The simulated robot in RViz moves according to the command

---

## Useful Commands

To launch the MQTT bridge:
```bash
ros2 launch mqtt_client standalone.launch.xml params_file:="/home/achref/ROS2_Humble/4_6_wheels_mobile_robot/src/my_robot_bringup/config/robot_joystick_bridge.yaml"
```

To echo messages on the ROS 2 topics:
```bash
ros2 topic echo /robot/control
ros2 topic echo /diff_drive_controller/cmd_vel
```

## Errors and logs when launching

No real-time kernel detected:

ROS 2 control can use real-time scheduling for precise hardware control, but most development machines use standard Linux kernels.
Fix: Install and boot into a real-time Linux kernel (PREEMPT_RT) if you need hard real-time performance for physical robots. For simulation, this is not needed.
Stereo not supported and OpenGL info in RViz:

RViz checks for stereo rendering and reports if it’s not available. OpenGL version info is just for reference.
Fix: Stereo rendering requires specific hardware and drivers. If you don’t need stereo vision, you can ignore this message.
