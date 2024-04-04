cmake_minimum_required(VERSION 3.8)
project(multi_robot_move)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)


set(DEPENDENCIES
  "rclcpp"
  "geometry_msgs"
  "nav2_msgs"
  "rclcpp_action"
)

install(TARGETS ${EXEC_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/utils/NavigationClient.py
  scripts/utils/DataRobots.py
  scripts/utils/PoseUtils.py
  scripts/nav_pose.py
  scripts/nav_pose_v2.py
  scripts/nav_through_poses.py
  scripts/nav_through_poses_v2.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rclcpp)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(nav2_msgs)
ament_export_dependencies(rclcpp_action)
ament_package()