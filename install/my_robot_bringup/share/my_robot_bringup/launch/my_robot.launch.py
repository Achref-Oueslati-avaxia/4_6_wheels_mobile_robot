# my_robot.launch.py  (edited for Humble compatibility)
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from launch_ros.actions import Node
from launch.actions import TimerAction
import os

def generate_launch_description():
    robot_description_path = get_package_share_path('my_robot_description')
    robot_bringup_path = get_package_share_path('my_robot_bringup')
    
    urdf_path = os.path.join(robot_description_path, 'urdf', 'my_robot.urdf')
    rviz_config_path = os.path.join(robot_description_path, 'rviz', 'urdf_config.rviz')

    # generate robot_description from xacro
    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    # path to controllers YAML (controller_manager params)
    robot_controllers = os.path.join(robot_bringup_path, 'config', 'my_robot_controllers.yaml')

    # publish robot description (robot_state_publisher)
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description': robot_description}],
    )

    # pass both the controllers YAML and robot_description to the controller manager
    # (passing robot_description as parameter helps Humble setups that expect it;
    #  some distributions later recommend using the topic --- this is compatible)
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        # file path (YAML) + explicit robot_description parameter for compatibility
        parameters=[robot_controllers, {'robot_description': robot_description}],
    )


    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller"],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_path],
    )
 
    joystick_mqtt_to_twist_node = Node(
        package="my_robot_bringup",
        executable="joystick_mqtt_to_twist",
        name="joystick_mqtt_to_twist",
        output="screen",
    )

    # Uncomment below to use joystick_old.py instead:
    # joystick_old_node = Node(
    #     package="my_robot_bringup",
    #     executable="joystick_old",
    #     name="joystick_old",
    #     output="screen",
    # )

    return LaunchDescription([
        robot_state_publisher_node,
        control_node,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner,
        rviz_node,
        joystick_mqtt_to_twist_node,
        # joystick_old_node,  # Uncomment to launch joystick_old.py instead
    ])
