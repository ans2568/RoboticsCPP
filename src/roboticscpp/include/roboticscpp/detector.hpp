#pragma once

#ifndef _DETECTOR_HPP
#define _DETECTOR_HPP

#include "types.hpp"
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>

class Detector {
public:
  Detector(const std::string class_file_path,
           const std::string config_file_path, const std::string weights_path)
      : class_file_(class_file_path), config_file_(config_file_path),
        weights_file_(weights_path) {}

// TODO description all of them
  bool Init();
  bool LoadModel();

  void Detect(cv::Mat& image);

  std::vector<std::string> GetOutputsNames(const cv::dnn::Net &net);

  void RemoveLowConfidenceBoxes(cv::Mat &image,
                                const std::vector<cv::Mat> &outputs);

  void DrawResult(int class_id, float confidence, cv::Rect box, cv::Mat& image);

private:
  std::vector<std::string> classes_;
  std::vector<std::string> names_;

  std::string class_file_;
  std::string config_file_;
  std::string weights_file_;

  cv::dnn::Net net_;
  cv::Mat blob_;

  // TODO threshold parameter using config file or start entry point(main)
  double confidence_th_ = 0.7;
  double nms_th_ = 0.4;
};

#endif