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

        // Debug 图像发布（best-effort, depth=1），默认开启
        publish_debug_image_ = this->declare_parameter<bool>("publish_debug_image", true);
        debug_image_topic_ = this->declare_parameter<std::string>("debug_image_topic", "/debug/detection_image");
        if (publish_debug_image_) {
            auto debug_qos = rclcpp::SensorDataQoS().keep_last(1).best_effort();
            debug_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(debug_image_topic_, debug_qos);
            RCLCPP_INFO(this->get_logger(), "Debug image publisher enabled on %s", debug_image_topic_.c_str());
        } else {
            RCLCPP_INFO(this->get_logger(), "Debug image publisher disabled");
        }

        RCLCPP_INFO(this->get_logger(), "检测节点初始化完成 | 模型路径: %s", model_path.c_str());
    }

    void ImageSubscriber::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
            // 10Hz 发布节流：仅当距离上次发布超过100ms才发布
            const auto now = std::chrono::steady_clock::now();

            // [TEST] 帧率统计 - 每100帧计算平均帧率
            frame_count_++;
            if (frame_count_ == 1) {
                fps_start_time_ = now;
            } else if (frame_count_ % 100 == 0) {
                auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - fps_start_time_).count();
                double avg_fps = 100.0 * 1000.0 / elapsed;
                double avg_frame_ms = static_cast<double>(elapsed) / 100.0;  // 每帧耗时

                RCLCPP_INFO(
                    this->get_logger(),
                    "[TEST] 帧率统计: 最近100帧平均帧率 = %.2f FPS, 平均每帧耗时 = %.2f ms",
                    avg_fps,
                    avg_frame_ms
                );
                fps_start_time_ = now;  // 重置起始时间
            }
            if (last_publish_time_.time_since_epoch().count() != 0) {
                if (now - last_publish_time_ < publish_interval_) {
                    return; // 跳过本次发布
                }
            }
            
            // 转为 OpenCV 格式（零拷贝）
            auto cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
            const cv::Mat& image = cv_ptr->image;
            detectionArmor_.start_detection(image);

            std::vector<detection::ArmorData> armors = detectionArmor_.getdata();

            float optical_center_x = 0.0f;
            float optical_center_y = 0.0f;
            float delta_x = 0.0f;
            float delta_y = 0.0f;
            float flag = 0.0f;
            float target_x = 0.0f;
            float target_y = 0.0f;

            if (!armors.empty()) {
                detection::ArmorData target = armors[0];
                optical_center_x = target.optical_center.x;
                optical_center_y = target.optical_center.y;
                delta_x = target.delta_x;
                delta_y = target.delta_y;
                flag = float(target.flag);
                target_x = static_cast<float>(target.center_point.x);
                target_y = static_cast<float>(target.center_point.y);

                auto delta_msg = geometry_msgs::msg::Point();
                delta_msg.x = delta_x;
                delta_msg.y = delta_y;
                delta_msg.z = 0.0f;
                delta_publisher_->publish(delta_msg);

                auto target_info_msg = std_msgs::msg::Float32MultiArray();
                target_info_msg.data = {
                    target_x,
                    target_y,
                    optical_center_x,
                    optical_center_y,
                    delta_x,
                    delta_y,
                    flag
                };
                target_info_publisher_->publish(target_info_msg);

                RCLCPP_INFO(this->get_logger(),
                    "目标: (%.1f, %.1f) | 光心: (%.1f, %.1f) | 差值: (%.1f, %.1f) | 标志: %.1f",
                    target_x, target_y, optical_center_x, optical_center_y, delta_x, delta_y, flag);
                last_publish_time_ = now; // 记录发布时间
            }

            // 发布带检测框的调试图像（仅在有订阅者时，best-effort，不反压）
            if (publish_debug_image_ && debug_image_publisher_ && debug_image_publisher_->get_subscription_count() > 0) {
                auto dbg_msg = cv_bridge::CvImage(msg->header, "bgr8", image).toImageMsg();
                debug_image_publisher_->publish(*dbg_msg);
            }

        } catch (const cv::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "OpenCV异常: %s", e.what());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "检测回调异常: %s", e.what());
        }
    }
}
