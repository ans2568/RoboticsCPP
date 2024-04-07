#include "roboticscpp/image_subscriber.hpp"
#include <opencv2/highgui.hpp>

void ImageSubscriber::SetDetector(Detector detector) {
  detector_ = std::make_shared<Detector>(std::move(detector));
}

void ImageSubscriber::Callback(
    const std::shared_ptr<sensor_msgs::msg::Image> msg) {
  if (detector_ == nullptr) {
    std::cerr << "detector_ is nullptr. set detector to ImageSubscriber"
              << std::endl;
    return;
  } else {
    cv_ptr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    detector_->Detect(cv_ptr_->image);
    cv::imshow("Detected", cv_ptr_->image);
  }
}