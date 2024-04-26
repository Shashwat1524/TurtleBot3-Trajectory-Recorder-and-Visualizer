cmake_minimum_required(VERSION 3.8)
project(my_robot_trajectory_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()




# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(turtlebot3 REQUIRED)
find_package(turtlebot3_gazebo REQUIRED)
find_package(anscer_interface REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


add_executable(trajectory_node src/trajectory_publisher.cpp)
ament_target_dependencies(trajectory_node rclcpp anscer_interface visualization_msgs nav_msgs geometry_msgs)

add_executable(trajectory_reader_node src/trajectory_reader.cpp)
ament_target_dependencies(trajectory_reader_node rclcpp visualization_msgs nav_msgs geometry_msgs tf2_ros tf2_geometry_msgs)

install(TARGETS 
  trajectory_node
  trajectory_reader_node
  DESTINATION lib/${PROJECT_NAME}    
)

ament_package()
