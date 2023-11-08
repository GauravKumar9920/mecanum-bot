cmake_minimum_required(VERSION 3.0.2)
project(holonomic_controller)

find_package(catkin REQUIRED 
  COMPONENTS
    roscpp
    rospy
    std_msgs
    tf
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS 
  roscpp 
  rospy 
  std_msgs
)

add_executable(holonomic_odometry_node src/holonomic_odometry.cpp)
target_link_libraries(holonomic_odometry_node 
${catkin_LIBRARIES}
)

# add_executable(cmd_vel_odom src/cmd_vel_to_odom.cpp)
# target_link_libraries(cmd_vel_odom 
# ${catkin_LIBRARIES}
# )

add_executable(holonomic_controller_node src/holonomic_controller_node.cpp)
add_dependencies(holonomic_controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(holonomic_controller_node 
${catkin_LIBRARIES}
jsoncpp
)

install(TARGETS holonomic_controller_node
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
       )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)