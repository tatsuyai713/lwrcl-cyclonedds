#ifndef ROSTYPEIMAGEPUBLSUB_H_
#define ROSTYPEIMAGEPUBLSUB_H_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <functional>
#include <memory>
#include <string>

using namespace rclcpp;

class ROSTypeImagePubSub : public Node
{
public:
  ROSTypeImagePubSub(uint16_t domain_number);
  ROSTypeImagePubSub(std::string node_name);
  ROSTypeImagePubSub(std::shared_ptr<dds::domain::DomainParticipant> participant);
  virtual ~ROSTypeImagePubSub();

  // Override init and run methods from Node
  // void run() override;

  void init();

  // Callback function to publish data
  void callbackPublish(int test);

  // Callback function to subscribe data
  void callbackSubscribe(std::shared_ptr<sensor_msgs::msg::Image> message);

private:
  std::string publish_topic_name_;
  std::string subscribe_topic_name_;
  int interval_ms_;
  Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_ptr_;
  Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber_ptr_;
  TimerBase::SharedPtr timer_ptr_;
  std::function<void()> timer_callback_;
  int counter_;
};

#endif /* ROSTYPEIMAGEPUBLSUB_H_ */
