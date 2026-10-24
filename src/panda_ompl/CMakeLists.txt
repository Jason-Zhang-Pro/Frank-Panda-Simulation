cmake_minimum_required(VERSION 3.16)
project(panda_ompl)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ==================== 查找依赖 ====================

find_package(ament_cmake REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ompl REQUIRED)
find_package(moveit_core REQUIRED)
find_package(srdfdom REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)

add_library(panda_planning_engine src/planning_engine.cpp src/bspline_trajectory.cpp src/cartesian_servo.cpp)
set_target_properties(panda_planning_engine PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(panda_planning_engine PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OMPL_INCLUDE_DIRS})
target_link_directories(panda_planning_engine PUBLIC ${OMPL_LIBRARY_DIRS})
ament_target_dependencies(panda_planning_engine moveit_core urdf srdfdom moveit_msgs shape_msgs geometry_msgs trajectory_msgs)
target_link_libraries(panda_planning_engine ${OMPL_LIBRARIES})

# Torque tracking and gripper control, independent of the planner backend.
add_executable(control_node src/control_node.cpp)
target_include_directories(control_node PRIVATE include)
target_link_libraries(control_node Eigen3::Eigen)
ament_target_dependencies(control_node rclcpp sensor_msgs std_msgs trajectory_msgs rosgraph_msgs urdf)
install(TARGETS control_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

add_executable(planner_node src/planner_node.cpp)
ament_target_dependencies(planner_node rclcpp sensor_msgs std_msgs std_srvs tf2_msgs ament_index_cpp trajectory_msgs ros_gz_interfaces)
target_link_libraries(planner_node panda_planning_engine)
install(TARGETS planner_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS panda_planning_engine DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)



add_executable(startup_node src/startup_node.cpp)
ament_target_dependencies(startup_node rclcpp controller_manager_msgs ros_gz_interfaces sensor_msgs std_msgs)
install(TARGETS startup_node DESTINATION lib/${PROJECT_NAME})

add_executable(benchmark_node src/benchmark_node.cpp)
ament_target_dependencies(benchmark_node rclcpp)
target_link_libraries(benchmark_node panda_planning_engine)
install(TARGETS benchmark_node DESTINATION lib/${PROJECT_NAME})

add_executable(gazebo_trial_node src/gazebo_trial_node.cpp)
ament_target_dependencies(gazebo_trial_node rclcpp sensor_msgs trajectory_msgs tf2_msgs std_msgs std_srvs)
install(TARGETS gazebo_trial_node DESTINATION lib/${PROJECT_NAME})

ament_package()
