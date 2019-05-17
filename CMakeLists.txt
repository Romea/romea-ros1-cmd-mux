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
  std_msgs
  geometry_msgs
  four_wheel_steering_msgs
  ackermann_msgs
  diagnostic_updater)

find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_DEPS})

###################################
## catkin specific configuration ##
###################################
catkin_package(CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS})

###########
## build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(cmd_mux_nodelets src/cmd_mux_nodelets.cpp src/simple_cmd_mux_nodelets.cpp)
target_link_libraries(cmd_mux_nodelets ${catkin_LIBRARIES})

add_executable(four_wheel_steering_cmd_mux_node src/cmd_mux_node.cpp)
target_compile_definitions(four_wheel_steering_cmd_mux_node PRIVATE -DFOUR_WHEEL_STEERING)
target_link_libraries(four_wheel_steering_cmd_mux_node ${catkin_LIBRARIES})

add_executable(ackerman_steering_cmd_mux_node src/cmd_mux_node.cpp)
target_compile_definitions(ackerman_steering_cmd_mux_node PRIVATE -DACKERMANN_STEERING)
target_link_libraries(ackerman_steering_cmd_mux_node ${catkin_LIBRARIES})

add_executable(skid_steering_cmd_mux_node src/cmd_mux_node.cpp)
target_compile_definitions(skid_steering_cmd_mux_node PRIVATE -DSKID_STEERING)
target_link_libraries(skid_steering_cmd_mux_node ${catkin_LIBRARIES})

add_executable(four_wheel_steering_simple_cmd_mux_node src/simple_cmd_mux_node.cpp)
target_compile_definitions(four_wheel_steering_simple_cmd_mux_node PRIVATE -DFOUR_WHEEL_STEERING)
target_link_libraries(four_wheel_steering_simple_cmd_mux_node ${catkin_LIBRARIES})

add_executable(ackerman_steering_simple_cmd_mux_node src/simple_cmd_mux_node.cpp)
target_compile_definitions(ackerman_steering_simple_cmd_mux_node PRIVATE -DACKERMANN_STEERING)
target_link_libraries(ackerman_steering_simple_cmd_mux_node ${catkin_LIBRARIES})

add_executable(skid_steering_simple_cmd_mux_node src/simple_cmd_mux_node.cpp)
target_compile_definitions(skid_steering_simple_cmd_mux_node PRIVATE -DSKID_STEERING)
target_link_libraries(skid_steering_simple_cmd_mux_node ${catkin_LIBRARIES})


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
