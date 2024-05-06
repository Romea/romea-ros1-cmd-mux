// Copyright 2024 INRAE, French National Research Institute for Agriculture, Food and Environment
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

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


