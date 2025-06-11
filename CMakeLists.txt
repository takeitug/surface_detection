cmake_minimum_required(VERSION 3.8)
project(manipulability)


# 必要な依存パッケージ
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(joint_state_listener src/joint_state_listener.cpp)
ament_target_dependencies(joint_state_listener rclcpp sensor_msgs)

install(TARGETS
  joint_state_listener
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
