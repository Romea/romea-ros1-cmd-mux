#include "base_nodelet.hpp"

namespace romea
{

template < typename T>
class CmdMuxNodelet : public BaseNodelet<T>
{

public:

  CmdMuxNodelet();

  virtual ~CmdMuxNodelet()=default;

  virtual void onInit() override;

};


using FourWheelSteeringCmdMuxNodelet = CmdMuxNodelet<four_wheel_steering_msgs::FourWheelSteering>;
using AckermanSteeringCmdMuxNodelet = CmdMuxNodelet<ackermann_msgs::AckermannDrive>;
using SkidSteeringCmdMuxNodelet = CmdMuxNodelet<geometry_msgs::Twist>;

}
