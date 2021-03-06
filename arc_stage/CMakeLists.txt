cmake_minimum_required(VERSION 2.8.3)
project(arc_stage)
find_package(catkin REQUIRED
  COMPONENTS
    geometry_msgs
    nav_msgs
    roscpp
    sensor_msgs
    marker_msgs
    std_msgs
    std_srvs
    tf
    dynamic_reconfigure
    arc_msgs
)

find_package(Boost REQUIRED COMPONENTS system thread)

find_package(stage REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${STAGE_INCLUDE_DIRS}
)

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/StageRos.cfg
)

catkin_package()

# Declare a cpp executable
add_executable(arc_stage src/stageros.cpp)
set(${PROJECT_NAME}_extra_libs "")
if(UNIX AND NOT APPLE)
  set(${PROJECT_NAME}_extra_libs dl)
endif()
target_link_libraries(arc_stage
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${STAGE_LIBRARIES}
  ${${PROJECT_NAME}_extra_libs}
)
if(catkin_EXPORTED_TARGETS)
  add_dependencies(arc_stage ${catkin_EXPORTED_TARGETS} arc_msgs_generate_messages_cpp)
endif()

## Install

install(PROGRAMS scripts/upgrade-world.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS arc_stage
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY world
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
## Tests

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
endif()
