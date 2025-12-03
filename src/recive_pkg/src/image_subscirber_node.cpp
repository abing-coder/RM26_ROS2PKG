#include "recive_pkg/image_subscriber_node.hpp"

namespace armor_detection
{
    // 构造函数接收模型路径
    ImageSubscriber::ImageSubscriber(const std::string& model_path)
        : Node("image_subscriber"),
        model_path_copy_(model_path),
        detectionArmor_(model_path_copy_, false)
    {
        // 设置检测颜色: 0 = 红色, 1 = 蓝色
        detection::DetectionArmor::detect_color = 0;
        // 使用 sensor_data 初始化的 QoS（但我们显式将发布 target_delta 的 QoS 设为 reliable + transient_local）
        auto image_qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
        image_qos.best_effort(); // 保持摄像头订阅为 sensor_data 风格（通常 best-effort）

        // 创建订阅者（图像）
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_raw",
            image_qos,
            std::bind(&ImageSubscriber::imageCallback, this, std::placeholders::_1)
        );

        // target_delta 发布使用 reliable + transient_local（与订阅者对齐）
        auto delta_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local();
        delta_publisher_ = this->create_publisher<geometry_msgs::msg::Point>("target_delta", delta_qos);

        // target_info 也使用同样的持久化 QoS（如果接收方需要历史）
        target_info_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("target_info", delta_qos);

        RCLCPP_INFO(this->get_logger(), "检测节点初始化完成 | 模型路径: %s", model_path.c_str());
    }

    void ImageSubscriber::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
            // 10Hz 发布节流：仅当距离上次发布超过100ms才发布
            const auto now = std::chrono::steady_clock::now();
            if (last_publish_time_.time_since_epoch().count() != 0) {
                if (now - last_publish_time_ < publish_interval_) {
                    return; // 跳过本次发布
                }
            }
            // 转为 OpenCV 格式
            cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            cv::Mat image = cv_ptr->image;
            detectionArmor_.start_detection(image);

            std::vector<detection::ArmorData> armors = detectionArmor_.getdata();

            if (!armors.empty()) {
                detection::ArmorData target = armors[0];

                // 计算差值
                float optical_center_x = target.optical_center.x;
                float optical_center_y = target.optical_center.y;
                float delta_x = target.delta_x;
                float delta_y = target.delta_y;

                auto delta_msg = geometry_msgs::msg::Point();
                delta_msg.x = delta_x;
                delta_msg.y = delta_y;
                delta_msg.z = 0.0f;
                delta_publisher_->publish(delta_msg);

                auto target_info_msg = std_msgs::msg::Float32MultiArray();
                target_info_msg.data = {
                    static_cast<float>(target.center_point.x),
                    static_cast<float>(target.center_point.y),
                    optical_center_x,
                    optical_center_y,
                    delta_x,
                    delta_y
                };
                target_info_publisher_->publish(target_info_msg);

                RCLCPP_INFO(this->get_logger(),
                    "检测到目标 - 目标: (%.1f, %.1f) | 光心: (%.1f, %.1f) | 差值: (%.1f, %.1f)",
                    static_cast<float>(target.center_point.x), static_cast<float>(target.center_point.y),
                    optical_center_x, optical_center_y, delta_x, delta_y);
                last_publish_time_ = now; // 记录发布时间
            }

            cv::imshow("原始图像", cv_ptr->image);
            cv::waitKey(1);
        } catch (const cv::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "OpenCV异常: %s", e.what());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "检测回调异常: %s", e.what());
        }
    }
}