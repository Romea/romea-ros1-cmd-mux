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
