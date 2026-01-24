cmake_minimum_required(VERSION 3.8)
project(saber_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# 手动包含头文件目录
include_directories(
  include
  /usr/local/include/OpenXLSX
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(saber_joint_controller 
  src/saber_joint_controller.cpp
  src/Cybergear_control.cpp
  src/disturbance_observer.cpp
  src/saber_get_regression.cpp
)

add_executable(saber_dynamics_identification 
  src/saber_dynamics_identification.cpp
  src/Cybergear_control.cpp
  src/disturbance_observer.cpp
  src/saber_get_regression.cpp
)

add_executable(saber_gravity_compensation 
  src/saber_gravity_compensation.cpp
  src/Cybergear_control.cpp
  src/disturbance_observer.cpp
  src/saber_get_regression.cpp
)

# 处理非ROS包依赖
target_link_libraries(saber_joint_controller
  OpenXLSX
  Eigen3::Eigen
  )

target_link_libraries(saber_dynamics_identification
  OpenXLSX
  Eigen3::Eigen
  )

target_link_libraries(saber_gravity_compensation
  OpenXLSX
  Eigen3::Eigen
  )

# 处理ROS包依赖
ament_target_dependencies(saber_joint_controller 
  rclcpp
  rclcpp_action
  sensor_msgs
  trajectory_msgs
  control_msgs
)

ament_target_dependencies(saber_dynamics_identification 
  rclcpp
  rclcpp_action
  sensor_msgs
  trajectory_msgs
  control_msgs
)

ament_target_dependencies(saber_gravity_compensation 
  rclcpp
  rclcpp_action
  sensor_msgs
  trajectory_msgs
  control_msgs
)

install(TARGETS
  saber_joint_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  saber_dynamics_identification
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  saber_gravity_compensation
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
