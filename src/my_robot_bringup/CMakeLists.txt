cmake_minimum_required(VERSION 3.8)
project(my_robot_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)


# Install launch and config files
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python scripts as executables
install(
  PROGRAMS
    joystick_mqtt_to_twist/joystick_mqtt_to_twist.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME joystick_mqtt_to_twist
)

# install(
#   PROGRAMS
#     joystick_mqtt_to_twist/joystick_old.py
#   DESTINATION lib/${PROJECT_NAME}
#   RENAME joystick_old
# )

ament_package()
