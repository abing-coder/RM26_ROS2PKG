#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <string>

namespace recive_pkg
{
class DebugDisplayNode : public rclcpp::Node
{
public:
  DebugDisplayNode()
  : Node("debug_display_node")
  {
    image_topic_ = this->declare_parameter<std::string>("image_topic", "/debug/detection_image");
    window_name_ = this->declare_parameter<std::string>("window_name", "debug_image");
    skip_ = this->declare_parameter<int>("skip", 3);  // 仅显示每 skip 帧

    auto qos = rclcpp::SensorDataQoS().keep_last(1).best_effort();
    sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      image_topic_, qos, std::bind(&DebugDisplayNode::imageCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Debug display subscribed to %s", image_topic_.c_str());
    cv::startWindowThread();
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) const
  {
    try {
      if (++frame_count_ % skip_ != 0) {
        return;
      }
      // 使用 CvShare 避免拷贝；源编码默认 bgr8
      auto cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
      cv::imshow(window_name_, cv_ptr->image);
      cv::waitKey(1);
    } catch (const std::exception & e) {
      RCLCPP_WARN(this->get_logger(), "Display failed: %s", e.what());
    }
  }

  std::string image_topic_;
  std::string window_name_;
  int skip_{3};
  mutable uint64_t frame_count_{0};
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
};
}  // namespace recive_pkg

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<recive_pkg::DebugDisplayNode>();
  rclcpp::spin(node);
  cv::destroyAllWindows();
  rclcpp::shutdown();
  return 0;
}
