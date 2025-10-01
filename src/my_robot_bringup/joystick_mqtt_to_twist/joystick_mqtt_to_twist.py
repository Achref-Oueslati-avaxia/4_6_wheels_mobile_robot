#!/usr/bin/env python3
"""
ROS 2 node: Subscribes to /robot/control (std_msgs/String, JSON from MQTT),
parses joystick commands, and publishes geometry_msgs/TwistStamped to /diff_drive_controller/cmd_vel.

Mapping (numpad directions):
    {"direction":8,"speed":S}  => forward
    {"direction":2,"speed":S}  => backward
    {"direction":4,"speed":S}  => left turn
    {"direction":6,"speed":S}  => right turn
    {"direction":7,"speed":S}  => forward left
    {"direction":9,"speed":S}  => forward right
    {"direction":1,"speed":S}  => backward left
    {"direction":3,"speed":S}  => backward right
    {"direction":5,"speed":S} or {"direction":0,"speed":S} => idle (stop)
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped
import json

SCALE = 0.01  # Adjust this to match your robot's speed (e.g., 0.01*50 = 0.5 m/s)

class JoystickMqttToTwist(Node):
    def __init__(self):
        super().__init__('joystick_mqtt_to_twist')
        self.sub = self.create_subscription(String, '/robot/joystick_cmd', self.cb, 10)
        self.pub = self.create_publisher(TwistStamped, '/diff_drive_controller/cmd_vel', 10)
        self.get_logger().info('Listening on /robot/joystick_cmd, publishing to /diff_drive_controller/cmd_vel')

    def cb(self, msg):
        try:
            data = json.loads(msg.data)
            twist = TwistStamped()
            s = float(data.get('speed', 0)) * SCALE
            direction = int(data.get('direction', 5))
            # Numpad analogy:
            # 789
            # 456
            # 123
            # 5 and 0 are idle
            if direction == 8:  # forward
                twist.twist.linear.x = s
                twist.twist.angular.z = 0.0
            elif direction == 2:  # backward
                twist.twist.linear.x = -s
                twist.twist.angular.z = 0.0
            elif direction == 4:  # left turn
                twist.twist.linear.x = 0.0
                twist.twist.angular.z = s
            elif direction == 6:  # right turn
                twist.twist.linear.x = 0.0
                twist.twist.angular.z = -s
            elif direction == 7:  # forward left
                twist.twist.linear.x = s
                twist.twist.angular.z = s
            elif direction == 9:  # forward right
                twist.twist.linear.x = s
                twist.twist.angular.z = -s
            elif direction == 1:  # backward left
                twist.twist.linear.x = -s
                twist.twist.angular.z = s
            elif direction == 3:  # backward right
                twist.twist.linear.x = -s
                twist.twist.angular.z = -s
            else:  # idle (5 or 0)
                twist.twist.linear.x = 0.0
                twist.twist.angular.z = 0.0
            self.pub.publish(twist)
        except Exception as e:
            self.get_logger().warn(f'Failed to parse joystick msg: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = JoystickMqttToTwist()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
