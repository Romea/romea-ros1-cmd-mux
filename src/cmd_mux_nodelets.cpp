//romea
#include "cmd_mux_nodelets.hpp"
#include "subscriber_configuration.hpp"

//std
#include <deque>

//yaml
#include <yaml-cpp/yaml.h>


namespace romea {

template < typename T>
CmdMuxNodelet<T>::CmdMuxNodelet()
{

}


template < typename T>
void CmdMuxNodelet<T>::onInit()
{

  ros::NodeHandle &private_nh = this->getMTPrivateNodeHandle();
  ros::NodeHandle &nh = this->getMTNodeHandle();

//  nodelet::V_string argv= this->getMyArgv();


  std::string config_filename;
  if(private_nh.getParam("config_file",config_filename))
  {

    try{

      YAML::Node config = YAML::LoadFile(config_filename);
      YAML::Node publisher_node = config["publisher"];
      YAML::Node suscribers_node = config["subscribers"];

      this->publisher_ = nh.advertise<T>(publisher_node["topic"].as<std::string>(),1);

      std::deque<SubscriberConfiguration> configurations;
      for(const auto subscriber_node : suscribers_node)
      {
        SubscriberConfiguration configuration;
        configuration.topic =subscriber_node["topic"].as<std::string>();
        configuration.name =subscriber_node["name"].as<std::string>();
        configuration.timeout = subscriber_node["timeout"].as<double>();
        configuration.priority =  subscriber_node["priority"].as<unsigned char>();
        configurations.push_back(std::move(configuration));
      }

      std::sort(configurations.begin(),configurations.end());

      this->subscribers_.resize(configurations.size());
      for(size_t n=0; n<configurations.size();++n)
      {
        typename BaseNodelet<T>::SubscriberCallbackFunction f =  boost::bind(&CmdMuxNodelet<T>::publishCallback,this,_1,n);
        this->subscribers_[n].timeout.fromSec(configurations[n].timeout);
        this->subscribers_[n].sub = nh.subscribe(configurations[n].topic,1,f);

      }

    }
    catch(YAML::Exception & e)
    {
      ROS_ERROR_STREAM("Cmd mux init failed, check "<< config_filename <<" is well formatted : "<< e.what());
    }
  }
}

template class CmdMuxNodelet<four_wheel_steering_msgs::FourWheelSteering>;
template class CmdMuxNodelet<ackermann_msgs::AckermannDrive>;
template class CmdMuxNodelet<geometry_msgs::Twist>;

}

PLUGINLIB_EXPORT_CLASS(romea::FourWheelSteeringCmdMuxNodelet, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS(romea::AckermanSteeringCmdMuxNodelet, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS(romea::SkidSteeringCmdMuxNodelet, nodelet::Nodelet);
