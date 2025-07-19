#ifndef ROSTYPEIMAGEPUBLSUBEDGE_H_
#define ROSTYPEIMAGEPUBLSUBEDGE_H_

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <functional>
#include <memory>
#include <string>

using namespace rclcpp;

class ROSTypeImagePubSubEdge : public Node
{
public:
  ROSTypeImagePubSubEdge(uint16_t domain_number);
  ROSTypeImagePubSubEdge(const std::string &node_name);
  ROSTypeImagePubSubEdge(std::shared_ptr<dds::domain::DomainParticipant> participant);
  ROSTypeImagePubSubEdge(std::shared_ptr<dds::domain::DomainParticipant> participant, const std::string &node_name);
  virtual ~ROSTypeImagePubSubEdge();

  // Callback function to subscribe data
  void callbackSubscribe(std::shared_ptr<sensor_msgs::msg::Image> message);

private:
  void init();
  std::string publish_topic_name_;
  std::string subscribe_topic_name_;
  int interval_ms_;
  Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_ptr_;
  Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber_ptr_;
  TimerBase::SharedPtr timer_ptr_;
  int counter_;
  std::shared_ptr<sensor_msgs::msg::Image> edge_msg_;
};

#endif /* ROSTYPEIMAGEPUBLSUBEDGE_H_ */
