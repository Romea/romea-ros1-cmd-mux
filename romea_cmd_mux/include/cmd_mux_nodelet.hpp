#ifndef CmdMuxNodelet_HPP
#define CmdMuxNodelet_HPP


//ros
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <topic_tools/shape_shifter.h>
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

class CmdMuxNodelet : public nodelet::Nodelet
{

protected :

  using SubscriberCallbackFunction = boost::function<void(const topic_tools::ShapeShifter::ConstPtr & msg)>;
  using SubscriberMap = std::map<unsigned char, Subscriber> ;

public:

  CmdMuxNodelet();
  virtual ~CmdMuxNodelet()=default;
  virtual void onInit()override;


protected:


  void diagnosticCallback(ros::TimerEvent & event);

  void publishCallback(const topic_tools::ShapeShifter::ConstPtr &msg,
                       unsigned char priotity);

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
  std::string publisher_topic_name_;
  bool is_publisher_initialized_;

};

}

#endif
