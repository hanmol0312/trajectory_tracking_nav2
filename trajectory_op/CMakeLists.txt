cmake_minimum_required(VERSION 3.8)
project(trajectory_op)

# if(NOT CMAKE_CXX_STANDARD)
 set(CMAKE_CXX_STANDARD 17)
 set(CMAKE_CXX_STANDARD_REQUIRED ON)
# endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tutorial_interfaces REQUIRED)   
find_package(yaml-cpp REQUIRED)     
find_package(tf2_ros REQUIRED)   
find_package(tf2_geometry_msgs REQUIRED)   
find_package(nav2_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(traj_publisher_saver src/traj_pub_saver.cpp)
ament_target_dependencies(traj_publisher_saver rclcpp nav_msgs visualization_msgs tutorial_interfaces geometry_msgs yaml-cpp nav2_msgs)

add_executable(traj_publisher_reader src/traj_pub_reader.cpp)
ament_target_dependencies(traj_publisher_reader rclcpp nav_msgs visualization_msgs tutorial_interfaces std_msgs geometry_msgs yaml-cpp tf2_ros tf2_geometry_msgs)

add_executable(traj_client src/traj_client.cpp)
ament_target_dependencies(traj_client rclcpp nav_msgs visualization_msgs tutorial_interfaces std_msgs geometry_msgs yaml-cpp)



target_link_libraries(traj_publisher_saver yaml-cpp stdc++fs)

target_link_libraries(traj_publisher_reader yaml-cpp)

# target_link_libraries(traj_publisher_reader tf2_ros)


install(TARGETS
traj_publisher_saver
traj_publisher_reader
traj_client
DESTINATION lib/${PROJECT_NAME})
 


install(DIRECTORY params
DESTINATION share/${PROJECT_NAME}
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
