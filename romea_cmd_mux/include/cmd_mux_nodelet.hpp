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

#ifndef CmdMuxNodelet_HPP
#define CmdMuxNodelet_HPP


//ros
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <topic_tools/shape_shifter.h>
#include <romea_cmd_mux_msgs/Connect.h>
#include <romea_cmd_mux_msgs/Disconnect.h>

//local
#include "subscriber_diagnostic.hpp"
#include "subscriber.hpp"

//std
#include <map>
#include <mutex>


namespace romea
{

class CmdMuxNodelet : public nodelet::Nodelet
{

protected :

  using SubscriberCallbackFunction = boost::function<void(const topic_tools::ShapeShifter::ConstPtr & msg)>;
  using SubscriberMap = std::map<unsigned char, Subscriber> ;

public:

  CmdMuxNodelet();
  virtual ~CmdMuxNodelet()=default;

protected:

  virtual void onInit()override;

  void diagnosticCallback_(ros::TimerEvent & event);

  void publishCallback_(const topic_tools::ShapeShifter::ConstPtr &msg,
                       unsigned char priotity);

  bool hasHighestPriority_(SubscriberMap::iterator it ,const ros::Time & now);

  bool connectCallback_(romea_cmd_mux_msgs::Connect::Request  &request,
                        romea_cmd_mux_msgs::Connect::Response & response);

  bool disconnectCallback_(romea_cmd_mux_msgs::Disconnect::Request  &request,
                           romea_cmd_mux_msgs::Disconnect::Response & response);

protected :

  std::mutex mutex_;
  ros::Publisher publisher_;
  SubscriberMap subscribers_;
  ros::ServiceServer connect_service_;
  ros::ServiceServer disconnect_service_;
  std::string publisher_topic_name_;
  bool is_publisher_initialized_;

};

}

#endif
