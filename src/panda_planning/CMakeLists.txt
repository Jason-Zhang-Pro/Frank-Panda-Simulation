cmake_minimum_required(VERSION 3.16)
project(panda_planning)
find_package(ament_cmake REQUIRED)
# Resource-only package: all algorithm/control/startup nodes live in panda_ompl.
install(DIRECTORY launch worlds DESTINATION share/${PROJECT_NAME})
ament_package()
