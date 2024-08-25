cmake_minimum_required(VERSION 3.11.0)
project(f1tenth_code)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  visualization_msgs
  ackermann_msgs
  nav_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(scan_processor src/ScanProcessor.cpp)
target_link_libraries(scan_processor ${catkin_LIBRARIES})

add_executable(marker_publisher src/MarkerPublisher.cpp)
target_link_libraries(marker_publisher ${catkin_LIBRARIES})

add_executable(straight_drive src/GoStraight5Sec.cpp)
target_link_libraries(straight_drive ${catkin_LIBRARIES})