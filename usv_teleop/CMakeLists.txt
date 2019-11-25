cmake_minimum_required(VERSION 2.8.3)
project(usv_teleop)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  tf2_geometry_msgs
  visualization_msgs)
include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
    INCLUDE_DIRS
    LIBRARIES
    CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs
    DEPENDS
)

add_executable(usv_teleop src/usv_teleop.cpp)
target_link_libraries(usv_teleop ${catkin_LIBRARIES})
add_dependencies(usv_teleop ${catkin_EXPORTED_TARGETS})

install(TARGETS usv_teleop
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION  ${CATKIN_PACKAGE_SHARE_DESTINATION}
)