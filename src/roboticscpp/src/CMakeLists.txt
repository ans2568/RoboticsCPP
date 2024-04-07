add_library(LIBS SHARED
    image_subscriber.cpp
    detector.cpp
)

ament_target_dependencies(LIBS rclcpp std_msgs sensor_msgs cv_bridge image_transport OpenCV)
target_link_libraries(LIBS ${OpenCV_LIBS})