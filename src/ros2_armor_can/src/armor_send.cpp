#include <rclcpp/rclcpp.hpp>
#include "robotcontrol/bmcan_bus.hpp"
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <chrono>
#include <vector>
#include <functional>
#include <thread>
#include <cstring>
#include <limits>

class ArmorSend : public rclcpp::Node {
public:
    ArmorSend() : Node("armor_send_node"), channelhandle(nullptr) {
        RCLCPP_INFO(this->get_logger(), "%s 节点已启动.", this->get_name());

        // 打开CAN通道（注意：请确认库的 open 签名是否需要传指针/引用）
        BM_NotificationHandle result = canbus.open(channelhandle, "BM-CANFD-X1(5850)");
        if (result == nullptr) {
            RCLCPP_ERROR(this->get_logger(), "CAN设备打开失败!");
        } else {
            RCLCPP_INFO(this->get_logger(), "CAN设备已打开");
        }
        RCLCPP_INFO(this->get_logger(), "channelhandle=%p notification=%p", (void*)channelhandle, (void*)result);

        // 订阅目标差值：显式使用与发布方一致的 QoS（RELIABLE + TRANSIENT_LOCAL）
        // auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local();
        // delta_subscription_ = this->create_subscription<geometry_msgs::msg::Point>(
        //     "target_delta",
        //     qos,
        //     [this](const geometry_msgs::msg::Point::SharedPtr msg) {
        //         try {
        //             RCLCPP_INFO(this->get_logger(), "delta_callback ENTER: x=%.3f y=%.3f", msg->x, msg->y);
        //             // 将发送放到后台线程，避免阻塞 DDS 回调线程
        //             double fx = msg->x;
        //             double fy = msg->y;
                    
        //             std::thread send_thread([this, fx, fy]() {
        //                 try {
        //                     this->send_motor_cmd(static_cast<int16_t>(fx), static_cast<int16_t>(fy), 0, 0);
        //                 } catch (const std::exception& e) {
        //                     RCLCPP_ERROR(this->get_logger(), "后台发送异常: %s", e.what());
        //                 } catch (...) {
        //                     RCLCPP_ERROR(this->get_logger(), "后台发送未知异常");
        //                 }
        //             });
        //             send_thread.detach();
        //         } catch (const std::exception &e) {
        //             RCLCPP_ERROR(this->get_logger(), "delta 回调异常: %s", e.what());
        //         } catch (...) {
        //             RCLCPP_ERROR(this->get_logger(), "delta 回调未知异常");
        //         }
        //     }
        // );

        // target_info 也显式 QoS（如果你需要接收历史也可设置 transient_local）
        target_info_subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "target_info",
            rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local(),
            std::bind(&ArmorSend::target_info_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "ros2_can 节点已启动 - 等待接收检测数据");
    }

    ~ArmorSend() {
        try {
            if (channelhandle != nullptr) {
                canbus.close(channelhandle);
                RCLCPP_INFO(this->get_logger(), "CAN设备已关闭");
            } else {
                RCLCPP_INFO(this->get_logger(), "channelhandle 无效，跳过关闭");
            }
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "关闭 CAN 设备发生异常: %s", e.what());
        }
    }

    // 发送电机控制命令（将实际发送放在此函数中）
    void send_motor_cmd(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4) {
        if (channelhandle == nullptr) {
            RCLCPP_ERROR(this->get_logger(), "channelhandle 无效，无法发送 CAN 数据");
            return;
        }

        uint8_t can_send_data[8];
        std::memset(can_send_data, 0, sizeof(can_send_data));
        can_send_data[0] = (motor1 >> 8) & 0xFF;
        can_send_data[1] = motor1 & 0xFF;
        can_send_data[2] = (motor2 >> 8) & 0xFF;
        can_send_data[3] = motor2 & 0xFF;
        can_send_data[4] = (motor3 >> 8) & 0xFF;
        can_send_data[5] = motor3 & 0xFF;
        

        RCLCPP_INFO(this->get_logger(), "准备发送自瞄偏移量: x=%d, y=%d, flag=%d", motor1, motor2, motor3);

        try {
            // 注意：请以 bmcan 库文档为准，下面假设 can_send(handle, id, data, len) 返回 0 表示成功
            int data_len = static_cast<int>(sizeof(can_send_data));
            auto ret = canbus.can_send(channelhandle, cantx_id, can_send_data, data_len);
            RCLCPP_INFO(this->get_logger(), "can_send 返回: %d", static_cast<int>(ret));
            if (ret != 0) {
                RCLCPP_ERROR(this->get_logger(), "can_send 发生错误，返回码: %d", static_cast<int>(ret));
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "发送自瞄偏移量抛出异常: %s", e.what());
        } catch (...) {
            RCLCPP_ERROR(this->get_logger(), "发送自瞄偏移量抛出未知异常");
        }
    }

private:
    BMCANTool canbus;
    BM_ChannelHandle channelhandle;
    const int cantx_id = 0x520;
    const int canrx_id = 0x101;

    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr delta_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr target_info_subscription_;
    rclcpp::TimerBase::SharedPtr timer_;

    void delta_callback(const geometry_msgs::msg::Point::SharedPtr msg) {
        // 目前不用（我们在 lambda 中直接处理），保留以备将来使用
    }

    void target_info_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() >= 7) {
            double delta_x = msg->data[4];
            double delta_y = msg->data[5];
            double flag = msg->data[6];
            RCLCPP_INFO(this->get_logger(),
                "目标:(%.1f,%.1f) 光心:(%.1f,%.1f) 差值:(%.1f,%.1f) flag:%.lf",
                msg->data[0], msg->data[1], msg->data[2], msg->data[3], delta_x, delta_y, flag);
            // 异步发送，避免阻塞回调线程
            std::thread([this, delta_x, delta_y, flag]() {
                this->send_motor_cmd(
                    static_cast<int16_t>(delta_x),
                    static_cast<int16_t>(delta_y),
                    static_cast<int16_t>(flag),
                    0
                );
            }).detach();
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto armor_send_node = std::make_shared<ArmorSend>();
    rclcpp::spin(armor_send_node);
    rclcpp::shutdown();
    return 0;
}