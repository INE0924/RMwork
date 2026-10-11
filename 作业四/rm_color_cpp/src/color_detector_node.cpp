#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>

class ColorDetectorNode : public rclcpp::Node {
public:
  ColorDetectorNode() : Node("color_detector_node") {
    show_windows_ = declare_parameter<bool>("show_windows", true);
    min_area_ = declare_parameter<double>("min_area", 400.0);
    red_publisher_ = create_publisher<sensor_msgs::msg::Image>(
      "/color/red_mask", rclcpp::SensorDataQoS());
    blue_publisher_ = create_publisher<sensor_msgs::msg::Image>(
      "/color/blue_mask", rclcpp::SensorDataQoS());
    detected_publisher_ = create_publisher<sensor_msgs::msg::Image>(
      "/color/detected", rclcpp::SensorDataQoS());
    subscription_ = create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_raw", rclcpp::SensorDataQoS(),
      std::bind(&ColorDetectorNode::on_image, this, std::placeholders::_1));

    if (show_windows_) {
      cv::namedWindow("Original", cv::WINDOW_NORMAL);
      cv::namedWindow("Threshold - Red HSV", cv::WINDOW_NORMAL);
      cv::namedWindow("Threshold - Blue BGR", cv::WINDOW_NORMAL);
      cv::namedWindow("Detected", cv::WINDOW_NORMAL);
    }
    RCLCPP_INFO(get_logger(), "Subscribed to /camera/image_raw");
  }

  ~ColorDetectorNode() override {
    if (show_windows_) cv::destroyAllWindows();
  }

private:
  // Method 1: HSV thresholds. Red wraps around both ends of OpenCV hue [0, 179].
  static cv::Mat red_hsv_mask(const cv::Mat &bgr) {
    cv::Mat hsv, low_red, high_red, mask;
    cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, cv::Scalar(0, 100, 65),
                cv::Scalar(10, 255, 255), low_red);
    cv::inRange(hsv, cv::Scalar(170, 100, 65),
                cv::Scalar(179, 255, 255), high_red);
    cv::bitwise_or(low_red, high_red, mask);
    return mask;
  }

  // Method 2: BGR channel difference. This also works for bright cyan-blue
  // (e.g., a cyan phone screen), where B and G can both be high.
  static cv::Mat blue_bgr_mask(const cv::Mat &bgr) {
    std::vector<cv::Mat> channels;
    cv::split(bgr, channels);
    cv::Mat b_minus_r, stronger_than_red, enough_blue, mask;
    cv::subtract(channels[0], channels[2], b_minus_r);
    cv::threshold(b_minus_r, stronger_than_red, 60, 255, cv::THRESH_BINARY);
    cv::threshold(channels[0], enough_blue, 100, 255, cv::THRESH_BINARY);
    cv::bitwise_and(stronger_than_red, enough_blue, mask);
    return mask;
  }

  static void remove_noise(cv::Mat &mask) {
    const cv::Mat kernel = cv::getStructuringElement(
      cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
  }

  void mark_contours(const cv::Mat &mask, cv::Mat &image,
                     const std::string &label, const cv::Scalar &color) const {
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask.clone(), contours, cv::RETR_EXTERNAL,
                     cv::CHAIN_APPROX_SIMPLE);
    for (const auto &contour : contours) {
      if (cv::contourArea(contour) < min_area_) continue;
      const cv::Rect rect = cv::boundingRect(contour);
      cv::rectangle(image, rect, color, 2);
      const cv::Point caption_at(rect.x, std::max(22, rect.y - 7));
      cv::putText(image, label, caption_at,
                  cv::FONT_HERSHEY_SIMPLEX, 0.7, color, 2);
    }
  }

  void publish_mask(const cv::Mat &mask, const std_msgs::msg::Header &header,
                    const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr &pub) {
    auto message = cv_bridge::CvImage(
      header, sensor_msgs::image_encodings::MONO8, mask).toImageMsg();
    pub->publish(*message);
  }

  void on_image(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
    cv::Mat image;
    try {
      image = cv_bridge::toCvCopy(msg,
        sensor_msgs::image_encodings::BGR8)->image;
    } catch (const cv_bridge::Exception &e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge failed: %s", e.what());
      return;
    }

    cv::Mat red_mask = red_hsv_mask(image);
    cv::Mat blue_mask = blue_bgr_mask(image);
    remove_noise(red_mask);
    remove_noise(blue_mask);

    cv::Mat detected = image.clone();
    mark_contours(red_mask, detected, "RED", cv::Scalar(0, 0, 255));
    mark_contours(blue_mask, detected, "BLUE", cv::Scalar(255, 100, 0));

    publish_mask(red_mask, msg->header, red_publisher_);
    publish_mask(blue_mask, msg->header, blue_publisher_);
    detected_publisher_->publish(*cv_bridge::CvImage(
      msg->header, sensor_msgs::image_encodings::BGR8, detected).toImageMsg());

    if (show_windows_) {
      cv::imshow("Original", image);
      cv::imshow("Threshold - Red HSV", red_mask);
      cv::imshow("Threshold - Blue BGR", blue_mask);
      cv::imshow("Detected", detected);
      cv::waitKey(1);  // Required for OpenCV windows to process GUI events.
    }
  }

  bool show_windows_;
  double min_area_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr red_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr blue_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr detected_publisher_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ColorDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
