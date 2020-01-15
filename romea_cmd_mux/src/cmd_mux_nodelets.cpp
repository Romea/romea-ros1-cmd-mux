#include "cmd_mux_nodelets.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
template < typename T>
CmdMuxNodelet<T>::CmdMuxNodelet():
  mutex_(),
  publisher_(),
  subscribers_(),
  connect_service_()
{

}

//-----------------------------------------------------------------------------
template < typename T>
void CmdMuxNodelet<T>::onInit()
{

  ros::NodeHandle private_nh = getMTPrivateNodeHandle();

  publisher_ = private_nh.advertise<T>("cmd_out",1);
  connect_service_ = private_nh.advertiseService("connect",&CmdMuxNodelet<T>::connectCallback_,this);
  disconnect_service_ = private_nh.advertiseService("disconnect",&CmdMuxNodelet<T>::disconnectCallback_,this);
}


//-----------------------------------------------------------------------------
template < typename T>
void CmdMuxNodelet<T>::diagnosticCallback(ros::TimerEvent & event)
{

}

//-----------------------------------------------------------------------------
template < typename T>
bool CmdMuxNodelet<T>::connectCallback_(romea_cmd_mux_msgs::Connect::Request  &request,
                                        romea_cmd_mux_msgs::Connect::Response & /*response*/)
{
  std::lock_guard<std::mutex> lock(mutex_);

  auto lambda = [&](const std::pair<unsigned char, Subscriber> & s)
  {
    return s.second.sub.getTopic()==request.topic;
  };


  auto itTopic = std::find_if(subscribers_.begin(),
                              subscribers_.end(),
                              lambda);

  auto itPriority = subscribers_.find(request.priority);

  if(itTopic==subscribers_.end() && itPriority == subscribers_.end())
  {
    auto & subscriber = subscribers_[request.priority];
    SubscriberCallbackFunction f =  boost::bind(&CmdMuxNodelet<T>::publishCallback,this,_1,request.priority);
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
template < typename T>
bool CmdMuxNodelet<T>::disconnectCallback_(romea_cmd_mux_msgs::Disconnect::Request  &request,
                                           romea_cmd_mux_msgs::Disconnect::Response & /*response*/)
{
  std::lock_guard<std::mutex> lock(mutex_);


  auto lambda = [&](const std::pair<unsigned char, Subscriber> & s)
  {
    return s.second.sub.getTopic()==request.topic;
  };

  auto it = std::find_if(subscribers_.begin(),
                         subscribers_.end(),
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
template <typename T>
void CmdMuxNodelet<T>::publishCallback(const boost::shared_ptr<T const> & msg,unsigned char priotity)
{
  std::lock_guard<std::mutex> lock(mutex_);

  ros::Time now = ros::Time::now();
  SubscriberMap::iterator it = subscribers_.find(priotity);

  (*it).second.msg_stamp = now;

  if(hasHighestPriority(it,now))
  {
    publisher_.publish(msg);
  }
}

//-----------------------------------------------------------------------------
template < typename T>
bool CmdMuxNodelet<T>::hasHighestPriority(SubscriberMap::iterator it ,const ros::Time & now)
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


template class CmdMuxNodelet<four_wheel_steering_msgs::FourWheelSteering>;
template class CmdMuxNodelet<ackermann_msgs::AckermannDrive>;
template class CmdMuxNodelet<geometry_msgs::Twist>;

}

PLUGINLIB_EXPORT_CLASS(romea::FourWheelSteeringCmdMuxNodelet, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS(romea::AckermanSteeringCmdMuxNodelet, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS(romea::SkidSteeringCmdMuxNodelet, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS(romea::OmniSteeringCmdMuxNodelet, nodelet::Nodelet);

