#ifndef IMAGE_SUBSCRIBER_NODE_HPP
#define IMAGE_SUBSCRIBER_NODE_HPP
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <opencv2/opencv.hpp>
#include "detect.hpp"  // 统一检测入口
#include <string>
#include <chrono>
#include <rclcpp/qos.hpp>

namespace armor_detection
{
    
    class ImageSubscriber : public rclcpp::Node
    {
    public:
        // 修改1：构造函数接收模型路径和调试标志
        explicit ImageSubscriber( const std::string& model_path);
        
        cv_bridge::CvImagePtr cv_ptr;

    private:
        void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
        // 统一检测器
        detection::Detect detector_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
        
        // 添加发布者
        rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr delta_publisher_;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr target_info_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_publisher_;

        // ROS 消息对象成员变量（重用）
        geometry_msgs::msg::Point delta_msg_;
        std_msgs::msg::Float32MultiArray target_info_msg_;

        // 发布节流：仅以10Hz发布
        std::chrono::steady_clock::time_point last_publish_time_{};
        std::chrono::milliseconds publish_interval_{10}; // 10Hz

        // [TEST] 帧率统计 - 每100帧计算平均帧率
        uint64_t frame_count_{0};
        std::chrono::steady_clock::time_point fps_start_time_{};

        // Debug 配置
        bool publish_debug_image_{true};
        std::string debug_image_topic_{"/debug/detection_image"};

        // 目标检测数据成员变量
        float optical_center_x_{0.0f};
        float optical_center_y_{0.0f};
        float delta_x_{0.0f};
        float delta_y_{0.0f};
        float flag_{0.0f};
        float target_x_{0.0f};
        float target_y_{0.0f};
    };

}
#endif // IMAGE_SUBSCRIBER_NODE_HPP
