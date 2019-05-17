#include <ros/ros.h>
#include <nodelet/loader.h>
#include "cmd_mux_nodelets.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cmd_mux");

  nodelet::Loader nodelet;
  nodelet::M_string remap(ros::names::getRemappings());
  nodelet::V_string my_argv; //(argv + 1, argv + argc);
  std::string nodelet_name = ros::this_node::getName();

#ifdef FOUR_WHEEL_STEERING
  std::cout << "FOUR_WHEEL_STEERING "<< std::endl;
//  nodelet.load(nodelet_name, "romea_command_mux/FourWheelSteeringCmdMuxNodelet", remap, my_argv);
#endif

#ifdef ACKERMANN_STEERING
  std::cout << "ACKERMANN_STEERING "<< std::endl;
//  nodelet.load(nodelet_name, "romea_command_mux/AckermannCmdMuxNodelet", remap, my_argv);
#endif

#ifdef SKID_STEERING
  std::cout << "SKID_STEERING "<< std::endl;
//  nodelet.load(nodelet_name, "romea_command_mux/AckermannCmdMuxNodelet", remap, my_argv);
#endif

  ros::spin();
}


