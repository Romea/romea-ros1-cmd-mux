#include "simple_cmd_mux_nodelets.hpp"

namespace {
const double DEFAULT_TIMEOUT =0.5;
}

namespace romea {

template < typename T>
SimpleCmdMuxNodelet<T>::SimpleCmdMuxNodelet()
{

}


template < typename T>
void SimpleCmdMuxNodelet<T>::onInit()
{

  ros::NodeHandle &private_nh = this->getMTPrivateNodeHandle();
  ros::NodeHandle &nh = this->getMTNodeHandle();


  double lowest_priority_input_timeout;
  private_nh.param("lowest_priority_input_timeout",lowest_priority_input_timeout,DEFAULT_TIMEOUT);

  double highest_priority_input_timeout;
  private_nh.param("highest_priority_input_timeout",highest_priority_input_timeout,DEFAULT_TIMEOUT);


  this->subscribers_.resize(2);
  this->publisher_ = nh.advertise<T>("output",1);

  {
    typename BaseNodelet<T>::SubscriberCallbackFunction f =  boost::bind(&SimpleCmdMuxNodelet<T>::publishCallback,this,_1,0);
    this->subscribers_[0].timeout.fromSec(lowest_priority_input_timeout);
    this->subscribers_[0].sub = nh.subscribe("lowest_priority_input",1,f);
  }

  {
    typename BaseNodelet<T>::SubscriberCallbackFunction f =  boost::bind(&SimpleCmdMuxNodelet<T>::publishCallback,this,_1,1);
    this->subscribers_[1].timeout.fromSec(highest_priority_input_timeout);
    this->subscribers_[1].sub = nh.subscribe("lowest_priority_input",1,f);
  }
}

template class SimpleCmdMuxNodelet<four_wheel_steering_msgs::FourWheelSteering>;
template class SimpleCmdMuxNodelet<ackermann_msgs::AckermannDrive>;
template class SimpleCmdMuxNodelet<geometry_msgs::Twist>;
}

PLUGINLIB_EXPORT_CLASS(romea::FourWheelSteeringSimpleCmdMuxNodelet, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS(romea::AckermanSteeringSimpleCmdMuxNodelet, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS(romea::SkidSteeringSimpleCmdMuxNodelet, nodelet::Nodelet);
