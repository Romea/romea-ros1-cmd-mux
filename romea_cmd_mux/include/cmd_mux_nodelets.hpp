#ifndef CmdMuxNodelet_HPP
#define CmdMuxNodelet_HPP


//ros
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <geometry_msgs/Twist.h>
#include <four_wheel_steering_msgs/FourWheelSteering.h>
#include <ackermann_msgs/AckermannDrive.h>
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

template < typename T>
class CmdMuxNodelet : public nodelet::Nodelet
{

protected :

  using SubscriberCallbackFunction = boost::function<void(const boost::shared_ptr<T const> & msg)>;
  using SubscriberMap = std::map<unsigned char, Subscriber> ;

public:

  CmdMuxNodelet();
  virtual ~CmdMuxNodelet()=default;
  virtual void onInit()override;


protected:


  void diagnosticCallback(ros::TimerEvent & event);

  void publishCallback(const boost::shared_ptr<T const> & msg,unsigned char priotity);

  bool hasHighestPriority(SubscriberMap::iterator it ,const ros::Time & now);

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

};

using FourWheelSteeringCmdMuxNodelet = CmdMuxNodelet<four_wheel_steering_msgs::FourWheelSteering>;
using AckermanSteeringCmdMuxNodelet = CmdMuxNodelet<ackermann_msgs::AckermannDrive>;
using SkidSteeringCmdMuxNodelet = CmdMuxNodelet<geometry_msgs::Twist>;

}

#endif
