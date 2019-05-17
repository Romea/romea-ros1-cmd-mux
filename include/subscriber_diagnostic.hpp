#ifndef SubscriberDiagnostic_HPP
#define SubscriberDiagnostic_HPP

//ros
#include <ros/ros.h>
#include <diagnostic_updater/diagnostic_updater.h>


namespace romea
{





//class  DiagnosticSubscriber : public diagnostic_updater::DiagnosticTask
//{

//public:


//  DiagnosticSubscriber(const std::string &name, Subscriber * subscriber);

//  void update(const ros::Time & now);

//  void reset();

//  bool isOk() const;

//  virtual void run(diagnostic_updater::DiagnosticStatusWrapper &stat) override;

//private:

//   Subscriber * subscriber_;
//   mutable std::mutex mutex_;
//};


} // namespace cmd_mux




#endif // TOPIC_HANDLE_H
