#include "roboticscpp/types.hpp"
#include "roboticscpp/detector.hpp"
#include "roboticscpp/image_subscriber.hpp"
#include <rclcpp/logging.hpp>

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exec;
  auto node = rclcpp::Node("main");
  auto main_logger = node.get_logger();
  if (argc < 1) {
    RCLCPP_ERROR(main_logger, "declare config yaml file path");
    return -1;
  }
  std::string yaml_file_path = argv[1];
  RCLCPP_INFO(main_logger, "yaml_file_path : %s", argv[1]);
  std::ifstream fin(yaml_file_path);
  if (!fin.is_open()) {
    RCLCPP_ERROR(main_logger, "Failed to open YAML file : %s", argv[1]);
    return -1;
  }
  YAML::Node yaml_config = YAML::Load(fin);
  std::string coco_names = yaml_config["coco_names"].as<std::string>();
  std::string model_config = yaml_config["model_config"].as<std::string>();
  std::string weights = yaml_config["weights"].as<std::string>();
  double confidence_th = yaml_config["confidence_th"].as<double>();
  double nms_th = yaml_config["nms_th"].as<double>();
  std::string image_topic = yaml_config["image_topic"].as<std::string>();

  Detector detector(coco_names, model_config, weights, confidence_th, nms_th);
  if (!detector.Init()) {
    RCLCPP_ERROR(main_logger, "Failed to initialize Detector class");
    return -1;
  }
  auto image_sub = std::make_shared<ImageSubscriber>("image_subscriber", image_topic);
  image_sub->SetDetector(detector);

  exec.add_node(image_sub);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}