cmake_minimum_required(VERSION 2.8.3)
project(romea_cmd_mux)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

#######################
## Find dependencies ##
#######################
set(${PROJECT_NAME}_CATKIN_DEPS
  roscpp
  roslib
  nodelet
  topic_tools
  diagnostic_updater
  romea_cmd_mux_msgs)

find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_DEPS})

###################################
## catkin specific configuration ##
###################################
catkin_package(CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS})

###########
## build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(romea_cmd_mux_nodelet src/cmd_mux_nodelet.cpp)
target_link_libraries(romea_cmd_mux_nodelet ${catkin_LIBRARIES})

add_executable(cmd_mux src/cmd_mux_node.cpp)
target_link_libraries(cmd_mux ${catkin_LIBRARIES})


#foreach(dir launch config)
#    install(DIRECTORY ${dir}/
#        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
#endforeach()

#install(DIRECTORY include/
#   DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#)

#if (CATKIN_ENABLE_TESTING)
#    find_package(rostest REQUIRED)
#    add_rostest(test/system.test)
#endif()
