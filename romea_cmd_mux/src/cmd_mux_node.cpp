#include <ros/ros.h>
#include <nodelet/loader.h>
#include "cmd_mux_nodelet.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cmd_mux");

  nodelet::Loader nodelet;
  nodelet::M_string remap(ros::names::getRemappings());
  nodelet::V_string my_argv;
  std::string nodelet_name = ros::this_node::getName();
  nodelet.load(nodelet_name, "romea_cmd_mux/CmdMuxNodelet", remap, my_argv);
  ros::spin();
}


