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

#ifndef Subscriber_HPP
#define Subscriber_HPP

//ros
#include <ros/ros.h>
#include <diagnostic_updater/diagnostic_updater.h>

//std
#include <mutex>

namespace romea
{


struct Subscriber
{
  Subscriber():
    sub(),
    timeout(0.),
    msg_stamp(0.)
  {

  }

  ros::Subscriber sub;
  ros::Duration timeout;
  ros::Time msg_stamp;
};


}

#endif
