#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

using namespace std::chrono_literals;

class CameraNode : public rclcpp::Node {
public:
  CameraNode() : Node("camera_node") {
    // "0" means the first Linux camera; an http://.../video URL also works.
    const std::string source = declare_parameter<std::string>("source", "0");
    const int width = declare_parameter<int>("width", 640);
    const int height = declare_parameter<int>("height", 480);
    const int fps = declare_parameter<int>("fps", 30);

    const bool numeric = !source.empty() &&
      source.find_first_not_of("0123456789") == std::string::npos;
    file_source_ = !numeric && source.find("://") == std::string::npos;
    const bool opened = numeric ? camera_.open(std::stoi(source)) : camera_.open(source);
    if (!opened) {
      throw std::runtime_error("Cannot open camera source: " + source);
    }

    // These settings usually take effect for local USB webcams, but may not
    // be supported by network video streams.
    camera_.set(cv::CAP_PROP_FRAME_WIDTH, width);
    camera_.set(cv::CAP_PROP_FRAME_HEIGHT, height);
    publisher_ = create_publisher<sensor_msgs::msg::Image>(
      "/camera/image_raw", rclcpp::SensorDataQoS());

    const auto interval = std::chrono::milliseconds(1000 / std::max(1, fps));
    timer_ = create_wall_timer(interval, std::bind(&CameraNode::publish_frame, this));
    RCLCPP_INFO(get_logger(), "Camera opened: %s", source.c_str());
  }

private:
  void publish_frame() {
    cv::Mat frame;
    if (!camera_.read(frame) || frame.empty()) {
      if (file_source_) {
        // Loop a sample video; useful for checking the ROS pipeline without a webcam.
        camera_.set(cv::CAP_PROP_POS_FRAMES, 0);
        camera_.read(frame);
      }
      if (frame.empty()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "No frame from camera (check device, network stream and firewall)");
        return;
      }
    }

    // OpenCV BGR -> ROS sensor_msgs/Image, retaining width, height and encoding.
    std_msgs::msg::Header header;
    header.stamp = now();
    header.frame_id = "camera_frame";
    auto message = cv_bridge::CvImage(
      header, sensor_msgs::image_encodings::BGR8, frame).toImageMsg();
    publisher_->publish(*message);
  }

  cv::VideoCapture camera_;
  bool file_source_ = false;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<CameraNode>());
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("camera_node"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
