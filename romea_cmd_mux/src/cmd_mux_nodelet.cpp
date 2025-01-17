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

#include "cmd_mux_nodelet.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
CmdMuxNodelet::CmdMuxNodelet():
  mutex_(),
  publisher_(),
  subscribers_(),
  connect_service_(),
  is_publisher_initialized_(false)
{

}

//-----------------------------------------------------------------------------
void CmdMuxNodelet::onInit()
{

  ros::NodeHandle private_nh = getMTPrivateNodeHandle();
  connect_service_ = private_nh.advertiseService("connect",&CmdMuxNodelet::connectCallback_,this);
  disconnect_service_ = private_nh.advertiseService("disconnect",&CmdMuxNodelet::disconnectCallback_,this);
}


//-----------------------------------------------------------------------------
void CmdMuxNodelet::diagnosticCallback_(ros::TimerEvent & event)
{

}

//-----------------------------------------------------------------------------
bool CmdMuxNodelet::connectCallback_(romea_cmd_mux_msgs::Connect::Request  &request,
                                     romea_cmd_mux_msgs::Connect::Response & /*response*/)
{
  std::lock_guard<std::mutex> lock(mutex_);

  auto lambda = [&request](const std::pair<unsigned char, Subscriber> & s)
  {
    return s.second.sub.getTopic()==request.topic;
  };


  auto itTopic = std::find_if(std::cbegin(subscribers_),
                              std::cend(subscribers_),
                              lambda);

  auto itPriority = subscribers_.find(request.priority);

  if(itTopic==std::cend(subscribers_) && itPriority == std::cend(subscribers_))
  {
    auto & subscriber = subscribers_[request.priority];
    SubscriberCallbackFunction f =  boost::bind(&CmdMuxNodelet::publishCallback_,this,_1,request.priority);
    subscriber.timeout.fromSec(request.timeout);
    subscriber.sub = getMTNodeHandle().subscribe(request.topic,1,f);
    return true;
  }
  else
  {
    return itTopic==itPriority;
  }
}

//-----------------------------------------------------------------------------
bool CmdMuxNodelet::disconnectCallback_(romea_cmd_mux_msgs::Disconnect::Request  &request,
                                        romea_cmd_mux_msgs::Disconnect::Response & /*response*/)
{
  std::lock_guard<std::mutex> lock(mutex_);


  auto lambda = [&](const std::pair<unsigned char, Subscriber> & s)
  {
    return s.second.sub.getTopic()==request.topic;
  };

  auto it = std::find_if(std::cbegin(subscribers_),
                         std::cend(subscribers_),
                         lambda);

  if(it!=subscribers_.end())
  {
    subscribers_.erase(it);
    return true;
  }
  else
  {
    return false;
  }

}


//-----------------------------------------------------------------------------
void CmdMuxNodelet::publishCallback_(const topic_tools::ShapeShifter::ConstPtr &msg,
                                    unsigned char priotity)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if(!is_publisher_initialized_)
  {
    publisher_= msg->advertise(getNodeHandle(),"cmd_out",1);
    is_publisher_initialized_=true;
  }

  ros::Time now = ros::Time::now();
  auto it = subscribers_.find(priotity);
  (*it).second.msg_stamp = now;
  if(hasHighestPriority_(it,now))
  {
    publisher_.publish(msg);
  }
}

//-----------------------------------------------------------------------------
bool CmdMuxNodelet::hasHighestPriority_(SubscriberMap::iterator it ,const ros::Time & now)
{
  while(++it != subscribers_.end())
  {
    if(now-(*it).second.msg_stamp<(*it).second.timeout)
    {
      return false;
    }
  }
  return true;
}

}

PLUGINLIB_EXPORT_CLASS(romea::CmdMuxNodelet, nodelet::Nodelet);

