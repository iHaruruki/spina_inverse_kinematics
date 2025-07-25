cmake_minimum_required(VERSION 3.8)
project(spina_inverse_kinematics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdf REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(inverse_kinematics_node src/inverse_kinematics_node.cpp)
ament_target_dependencies(inverse_kinematics_node
  rclcpp
  geometry_msgs
  sensor_msgs
  tf2_ros
  urdf
  Eigen3
)

add_executable(target_pose_publisher_node src/target_pose_publisher.cpp)
ament_target_dependencies(target_pose_publisher_node
  rclcpp
  geometry_msgs
)

add_executable(forward_kinematics_node src/forward_kinematics_node.cpp)
ament_target_dependencies(forward_kinematics_node
  rclcpp
  geometry_msgs
  sensor_msgs
)

install(TARGETS
  inverse_kinematics_node
  target_pose_publisher_node
  forward_kinematics_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY urdf launch rviz
  DESTINATION share/${PROJECT_NAME}
)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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
