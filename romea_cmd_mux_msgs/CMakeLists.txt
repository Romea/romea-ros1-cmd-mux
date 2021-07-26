cmake_minimum_required(VERSION 3.0.2)
project(romea_cmd_mux_msgs)

set(${PROJECT_NAME}_CATKIN_DEPS std_msgs message_generation)
find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_DEPS})

add_service_files(
  DIRECTORY
   srv
  FILES
   Connect.srv
   Disconnect.srv
)


generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS})
