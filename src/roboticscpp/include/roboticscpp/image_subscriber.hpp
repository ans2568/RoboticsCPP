#pragma once

#ifndef _IMAGE_SUBSCRIBER_HPP
#define _IMAGE_SUBSCRIBER_HPP

#include "detector.hpp"
#include "types.hpp"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>

class ImageSubscriber : public rclcpp::Node {
public:
  ImageSubscriber(const std::string node_name, const std::string topic_name)
      : rclcpp::Node(node_name, topic_name) {
    RCLCPP_INFO(this->get_logger(), "ImageSubscriber node initialize");
    sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        topic_name, SensorQoS(),
        std::bind(&ImageSubscriber::Callback, this, std::placeholders::_1));
  }

  // TODO method description
  void Callback(const std::shared_ptr<sensor_msgs::msg::Image> msg);

  // TODO method description
  void SetDetector(Detector detector);

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_ = nullptr;
  std::shared_ptr<Detector> detector_ = nullptr;
  cv_bridge::CvImagePtr cv_ptr_ = nullptr;
};

#endif