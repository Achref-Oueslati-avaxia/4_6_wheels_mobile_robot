# #!/usr/bin/env python3
# """
# ROS 2 node: Subscribes to /robot/control (std_msgs/String, JSON from MQTT),
# parses joystick commands, and publishes geometry_msgs/TwistStamped to /diff_drive_controller/cmd_vel.

# Mapping:
# - {"axis":"V","direction":1,"speed":S}  => forward  (linear.x = +SCALE*S)
# - {"axis":"V","direction":0,"speed":S}  => backward (linear.x = -SCALE*S)
# - {"axis":"H","direction":1,"speed":S}  => right turn (angular.z = -SCALE*S)
# - {"axis":"H","direction":0,"speed":S}  => left turn  (angular.z = +SCALE*S)
# """
# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String
# from geometry_msgs.msg import TwistStamped
# import json

# SCALE = 0.01  # Adjust this to match your robot's speed (e.g., 0.01*50 = 0.5 m/s)

# class JoystickMqttToTwist(Node):
#     def __init__(self):
#         super().__init__('joystick_mqtt_to_twist')
#         self.sub = self.create_subscription(String, '/robot/joystick_cmd', self.cb, 10)
#         self.pub = self.create_publisher(TwistStamped, '/diff_drive_controller/cmd_vel', 10)
#         self.get_logger().info('Listening on /robot/joystick_cmd, publishing to /diff_drive_controller/cmd_vel')

#     def cb(self, msg):
#         try:
#             data = json.loads(msg.data)
#             twist = TwistStamped()
#             s = float(data.get('speed', 0)) * SCALE
#             if data.get('axis') == 'V':
#                 if data.get('direction') == 1:
#                     twist.twist.linear.x = s
#                 else:
#                     twist.twist.linear.x = -s
#                 twist.twist.angular.z = 0.0
#             elif data.get('axis') == 'H':
#                 twist.twist.linear.x = 0.0
#                 if data.get('direction') == 1:
#                     twist.twist.angular.z = -s
#                 else:
#                     twist.twist.angular.z = s
#             else:
#                 return  # Ignore unknown axis
#             self.pub.publish(twist)
#         except Exception as e:
#             self.get_logger().warn(f'Failed to parse joystick msg: {e}')

# def main(args=None):
#     rclpy.init(args=args)
#     node = JoystickMqttToTwist()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
