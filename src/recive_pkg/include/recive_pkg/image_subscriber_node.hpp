#ifndef IMAGE_SUBSCRIBER_NODE_HPP
#define IMAGE_SUBSCRIBER_NODE_HPP
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <opencv2/opencv.hpp>
#include "yolo_detection.hpp"  // 确保包含检测头文件
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
        // 修改3：将检测器作为成员变量       
        std::string model_path_copy_; // 保存可修改的模型路径，满足非 const 引用需求
        detection::DetectionArmor detectionArmor_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
        
        // 添加发布者
        rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr delta_publisher_;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr target_info_publisher_;

        // 发布节流：仅以10Hz发布
        std::chrono::steady_clock::time_point last_publish_time_{};
        std::chrono::milliseconds publish_interval_{10}; // 10Hz

        // [TEST] 帧率统计 - 每100帧计算平均帧率
        uint64_t frame_count_{0};
        std::chrono::steady_clock::time_point fps_start_time_{};
    };

}
#endif // IMAGE_SUBSCRIBER_NODE_HPP