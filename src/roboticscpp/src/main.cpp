#include "roboticscpp/types.hpp"
#include "roboticscpp/detector.hpp"
#include "roboticscpp/image_subscriber.hpp"
int main(int argc, char **argv) {
  Detector detector("../config/coco.names", "../config/yolov3.cfg",
                    "../config/yolov3.weights");
  if (!detector.Init()) {
    std::cerr << "Failed to initialize Detector class" << std::endl;
    return -1;
  }
  ImageSubscriber image_sub("image_subscriber", "/image_raw");
  image_sub.SetDetector(detector);
  return 0;
}