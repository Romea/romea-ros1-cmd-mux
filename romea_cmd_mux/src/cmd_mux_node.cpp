#include <ros/ros.h>
#include <nodelet/loader.h>
#include "cmd_mux_nodelets.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cmd_mux");

  nodelet::Loader nodelet;
  nodelet::M_string remap(ros::names::getRemappings());
  nodelet::V_string my_argv;
  std::string nodelet_name = ros::this_node::getName();

#ifdef FOUR_WHEEL_STEERING
 nodelet.load(nodelet_name, "romea_cmd_mux/FourWheelSteeringCmdMuxNodelet", remap, my_argv);
#endif

#ifdef ACKERMANN_STEERING
  nodelet.load(nodelet_name, "romea_cmd_mux/AckermannCmdMuxNodelet", remap, my_argv);
#endif

#ifdef SKID_STEERING
  nodelet.load(nodelet_name, "romea_cmd_mux/SkidSteeringCmdMuxNodelet", remap, my_argv);
#endif

  ros::spin();
}


