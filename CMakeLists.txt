cmake_minimum_required(VERSION 3.11.0)
project(f1tenth_code)

find_package(catkin REQUIRED COMPONENTS
    roslib
    roscpp
    tf2
    tf2_ros
    tf2_geometry_msgs
    ackermann_msgs
    nav_msgs
    sensor_msgs
    geometry_msgs
    interactive_markers
    visualization_msgs
    std_msgs
    message_generation

)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# 기존 노드들
add_executable(scan_processor src/ScanProcessor.cpp)
target_link_libraries(scan_processor ${catkin_LIBRARIES})

add_executable(marker_publisher src/MarkerPublisher.cpp)
target_link_libraries(marker_publisher ${catkin_LIBRARIES})

add_executable(straight_drive src/GoStraight5Sec.cpp)
target_link_libraries(straight_drive ${catkin_LIBRARIES})

# 추가된 point_follower 노드
add_executable(point_follower src/NodeControl.cpp)
target_link_libraries(point_follower ${catkin_LIBRARIES})

add_executable(straight_Left src/GoStraightTurnLeft.cpp)
target_link_libraries(straight_Left ${catkin_LIBRARIES})


add_executable(straight_Right src/GoStraightTurnRight.cpp)
target_link_libraries(straight_Right ${catkin_LIBRARIES})

# 라이다 쫒기
add_executable(lidar_tractor src/LidarTractDemo.cpp)
target_link_libraries(lidar_tractor ${catkin_LIBRARIES})


# 왼쪽 벽 주행
add_executable(wall_following_drive src/WallFollowingLeft.cpp)
target_link_libraries(wall_following_drive ${catkin_LIBRARIES})


# 벽 주행 (wall follow.py를 cpp로 포팅하였다.)
add_executable(wall_follow_node src/wall_follow.cpp)
target_link_libraries(wall_follow_node ${catkin_LIBRARIES})
