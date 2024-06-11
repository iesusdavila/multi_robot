cmake_minimum_required(VERSION 3.8)
project(multi_robot_master_slave)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(action_msgs REQUIRED)


set(DEPENDENCIES
  "rclpy"
  "geometry_msgs"
  "nav2_msgs"
  "lifecycle_msgs"
  "action_msgs"
)

install(TARGETS ${EXEC_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/utils/delegate_task.py
  scripts/utils/navigation_client.py
  scripts/utils/data_robots.py
  scripts/utils/pose_utils.py
  scripts/robot/robot.py
  scripts/robot/navigate_master.py
  scripts/robot/navigate_slave.py
  scripts/nav_master_slave.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rclpy)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(nav2_msgs)
ament_export_dependencies(lifecycle_msgs)
ament_export_dependencies(action_msgs)
ament_package()