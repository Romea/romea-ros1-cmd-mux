////local
//#include "subscriber_diagnostic.hpp"



//namespace romea
//{



//DiagnosticSubscriber::DiagnosticSubscriber(const std::string &name,
//                                           const std::string &topic,
//                                           const ros::Duration & duration)
//{

//}

//void DiagnosticSubscriber::update(const SubscriberState & state)
//{

//}

//void DiagnosticSubscriber::reset()
//{

//}

//bool DiagnosticSubscriber::isOk() const
//{

//}

//void DiagnosticSubscriber::run(diagnostic_updater::DiagnosticStatusWrapper &stat)
//{
//  std::lock_guard<std::mutex> lock(mutex_);

//  ros::Time now = ros::
//  if(value_.is_initialized())
//  {

//    if(*value_ < desired_value_-epsilon_)
//    {
//      stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, getName()+" too low.");
//    }
//    else if(*value_ > desired_value_+epsilon_)
//    {
//      stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, getName()+" too high.");
//    }
//    else
//    {
//      stat.summary(diagnostic_msgs::DiagnosticStatus::OK, getName()+" OK.");
//    }

//    stat.add(getName(), *value_);

//  }
//  else
//  {
//    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, getName()+" unknown.");
//    stat.add(getName(), "Unkown");
//  }
//}

//private:

//std::string topic_;
//bool isConnected_;
//ros::Duration timeout_;
//ros::Time previous_stamp_;

//mutable std::mutex mutex_;
//};




//} // namespace cmd_mux




//#endif // TOPIC_HANDLE_H
