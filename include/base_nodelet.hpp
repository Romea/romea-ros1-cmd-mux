#ifndef CmdMuxBaseNodelet_HPP
#define CmdMuxBaseNodelet_HPP


//ros
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <geometry_msgs/Twist.h>
#include <four_wheel_steering_msgs/FourWheelSteering.h>
#include <ackermann_msgs/AckermannDrive.h>

//local
#include "subscriber_diagnostic.hpp"
#include "subscriber.hpp"


namespace romea
{

template < typename T>
class BaseNodelet : public nodelet::Nodelet
{

protected :

  using SubscriberCallbackFunction = boost::function<void(const boost::shared_ptr<T const> & msg)>;

public:

  BaseNodelet():
    subscribers_(),
    publisher_()
  {

  }

  virtual ~BaseNodelet()=default;


  virtual void onInit() =0;


protected:


  void diagnosticCallback(ros::TimerEvent & event)
  {

  }

  void publishCallback(const boost::shared_ptr<T const> & msg,size_t index)
  {
    ros::Time now = ros::Time::now();
    subscribers_[index].msg_stamp=now;

    if(hasHighestPriority(index,now))
    {
      publisher_.publish(msg);
    }
  }

  bool hasHighestPriority(size_t index,const ros::Time & now)
  {
    while(++index < subscribers_.size())
    {
      if(now-subscribers_[index].msg_stamp<subscribers_[index].timeout)
      {
        return false;
      }
    }
    return true;
  }

protected :

  std::vector<Subscriber> subscribers_;
  ros::Publisher publisher_;

};


}

#endif
