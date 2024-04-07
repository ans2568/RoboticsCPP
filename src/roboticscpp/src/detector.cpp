#include "roboticscpp/detector.hpp"

// TODO LOGGING method
bool Detector::Init() {
  if (!class_file_.empty()) {
    std::ifstream ifs(class_file_.c_str());
    if (!ifs.is_open()) {
      CV_Error(cv::Error::StsError, "Class File " + class_file_ + " not found");
      return false;
    }

    std::string row;
    while (std::getline(ifs, row))
      classes_.push_back(row);
  } else {
    std::cerr << class_file_ + " is empty. see Detector()" << std::endl;
    return false;
  }
  if (!LoadModel())
    return false;
  return true;
}

bool Detector::LoadModel() {
  if (config_file_.empty()) {
    std::cerr << config_file_ + " is empty. see Detector()" << std::endl;
    return false;
  }
  if (weights_file_.empty()) {
    std::cerr << weights_file_ + " is empty. see Detector()" << std::endl;
    return false;
  }
  net_ =
      cv::dnn::dnn4_v20230620::readNetFromDarknet(config_file_, weights_file_);
  net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
  net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);

  return true;
}

void Detector::Detect(cv::Mat& image) {
  cv::dnn::blobFromImage(image, blob_, 1 / 255.0, cv::Size(412, 412),
                         cv::Scalar(0, 0, 0), true, false);
  net_.setInput(blob_);
  std::vector<cv::Mat> outputs;
  net_.forward(outputs, GetOutputsNames(net_));
  RemoveLowConfidenceBoxes(image, outputs);
}

std::vector<std::string> Detector::GetOutputsNames(const cv::dnn::Net &net) {
  if (names_.empty()) {
    std::vector<int> out_layers = net.getUnconnectedOutLayers();
    std::vector<std::string> layer_names = net.getLayerNames();

    names_.resize(out_layers.size());
    for (size_t i = 0; i < out_layers.size(); ++i)
      names_[i] = layer_names[out_layers[i] - 1];
  }
  return names_;
}

void Detector::RemoveLowConfidenceBoxes(cv::Mat &image,
                                        const std::vector<cv::Mat> &outputs) {
  std::vector<int> class_ids;
  std::vector<float> confidences;
  std::vector<cv::Rect> boxes;

  for (size_t i = 0; i < outputs.size(); ++i) {
    auto data = (float *)outputs[i].data;
    for (int row = 0; row < outputs[i].rows; ++row, data += outputs[i].cols) {
      cv::Mat scores = outputs[i].row(row).colRange(5, outputs[i].cols);
      cv::Point class_id_point;
      double confidence;
      cv::minMaxLoc(scores, 0, &confidence, 0, &class_id_point);
      if (confidence > confidence_th_) {
        int center_x = (int)(data[0] * image.cols);
        int center_y = (int)(data[1] * image.rows);
        int width = (int)(data[2] * image.cols);
        int height = (int)(data[3] * image.rows);
        int left = center_x - width / 2;
        int top = center_y - height / 2;

        class_ids.push_back(class_id_point.x);
        confidences.push_back((float)confidence);
        boxes.push_back(cv::Rect(left, top, width, height));
      }
    }
  }

  std::vector<int> indices;
  cv::dnn::dnn4_v20230620::NMSBoxes(boxes, confidences, confidence_th_, nms_th_,
                                    indices);
  for (size_t i = 0; i < indices.size(); ++i) {
    int idx = indices[i];
    DrawResult(class_ids[idx], confidences[idx], boxes[idx], image);
  }
}

void Detector::DrawResult(int class_id, float confidence, cv::Rect box,
                          cv::Mat &image) {
  int left = box.x;
  int top = box.y;
  int right = box.x + box.width;
  int bottom = box.y + box.height;
  cv::rectangle(image, cv::Point(left, top), cv::Point(right, bottom),
                cv::Scalar(0, 0, 255));
  std::string label = cv::format("%.2f", confidence);
  if (!classes_.empty()) {
    CV_Assert(class_id < (int)classes_.size());
    label = classes_[class_id] + ":" + label;
  }
  int baseline;
  auto label_size =
      cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
  top = std::max(top, label_size.height);
  cv::putText(image, label, cv::Point(left, top), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255));
}