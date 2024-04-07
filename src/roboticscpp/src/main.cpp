#include "roboticscpp/types.hpp"
#include "roboticscpp/detector.hpp"
#include "roboticscpp/image_subscriber.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exec;
  Detector detector("/RoboticsCPP/src/roboticscpp/config/coco.names", "/RoboticsCPP/src/roboticscpp/config/yolov3.cfg",
                    "/RoboticsCPP/src/roboticscpp/config/yolov3.weights");
  if (!detector.Init()) {
    std::cerr << "Failed to initialize Detector class" << std::endl;
    return -1;
  }
  auto image_sub = std::make_shared<ImageSubscriber>("image_subscriber", "/image_raw");
  image_sub->SetDetector(detector);
  exec.add_node(image_sub);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}