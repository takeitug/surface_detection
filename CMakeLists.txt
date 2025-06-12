cmake_minimum_required(VERSION 3.8)
project(manipulability)

# 必要な依存パッケージ
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# ライブラリのビルド
add_library(manipulability_inverselib
  src/inverse.cpp
)
target_include_directories(manipulability_inverselib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(manipulability_inverselib Eigen3)

# 実行ファイルのビルド
add_executable(joint_state_listener src/joint_state_listener.cpp)
target_link_libraries(joint_state_listener manipulability_inverselib)
ament_target_dependencies(joint_state_listener rclcpp sensor_msgs Eigen3)

# ヘッダファイルのインストール
install(DIRECTORY include/
  DESTINATION include/
)

# 実行ファイル・ライブラリのインストール
install(TARGETS
  joint_state_listener
  manipulability_inverselib
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
