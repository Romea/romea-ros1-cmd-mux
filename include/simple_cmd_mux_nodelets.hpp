#ifndef SimpleCmdMuxNodelet_HPP
#define SimpleCmdMuxNodelet_HPP


//local
#include "base_nodelet.hpp"

//std
#include <deque>


namespace romea
{

template < typename T>
class SimpleCmdMuxNodelet : public BaseNodelet<T>
{

public:

  SimpleCmdMuxNodelet();

  virtual ~SimpleCmdMuxNodelet()=default;

  virtual void onInit() override;

};

using FourWheelSteeringSimpleCmdMuxNodelet = SimpleCmdMuxNodelet<four_wheel_steering_msgs::FourWheelSteering>;
using AckermanSteeringSimpleCmdMuxNodelet = SimpleCmdMuxNodelet<ackermann_msgs::AckermannDrive>;
using SkidSteeringSimpleCmdMuxNodelet = SimpleCmdMuxNodelet<geometry_msgs::Twist>;

}

#endif
