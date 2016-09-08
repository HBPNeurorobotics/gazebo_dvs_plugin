cmake_minimum_required(VERSION 2.8.3)
project(gazebo_dvs_plugin)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  gazebo_plugins
  dvs_msgs
  roscpp
  cv_bridge
)

find_package(gazebo REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    gazebo_ros
    gazebo_plugins
    roscpp
    dvs_msgs
    cv_bridge
)

###########
## Build ##
###########

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(
  include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}/Paging
  ${OpenCV_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
    src/dvs_plugin.cpp
)
add_dependencies(${PROJECT_NAME}
    ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${GAZEBO_LIBRARIES}
    ${Boost_LIBRARIES}
    ${OGRE_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${OpenCV_LIBRARIES}
    CameraPlugin
)
message (STATUS "GAZEBO_LIBRARIES: " ${GAZEBO_LIBRARIES})
install (TARGETS ${PROJECT_NAME} DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
