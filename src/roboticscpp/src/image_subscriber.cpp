#include "roboticscpp/image_subscriber.hpp"
#include <opencv2/highgui.hpp>

void ImageSubscriber::SetDetector(Detector detector) {
  RCLCPP_INFO(this->get_logger(), "Image Subscriber set detector");
  detector_ = std::make_shared<Detector>(std::move(detector));
}

void ImageSubscriber::Callback(
    const std::shared_ptr<sensor_msgs::msg::Image> msg) {
  if (detector_ == nullptr) {
    RCLCPP_ERROR(this->get_logger(), "detector_ is nullptr. set detector to ImageSubscriber");
    return;
  } else {
    cv_ptr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    detector_->Detect(cv_ptr_->image);
    cv::imshow("Detected", cv_ptr_->image);
    cv::waitKey(1);
  }
}