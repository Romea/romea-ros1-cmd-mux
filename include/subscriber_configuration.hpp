#ifndef SubscriberConfiguration_HPP
#define SubscriberConfiguration_HPP

//std
#include <string>

namespace romea
{


struct SubscriberConfiguration
{
  SubscriberConfiguration():
    name(),
    topic(),
    timeout(0.),
    priority(0.)
  {
  }

  std::string name;
  std::string topic;
  double timeout;
  unsigned char priority;
};

bool operator<(const SubscriberConfiguration &c1, const SubscriberConfiguration & c2)
{
  c1.priority<c2.priority;
}

}




#endif // TOPIC_HANDLE_H
