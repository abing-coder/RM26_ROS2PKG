#include <rclcpp/rclcpp.hpp>
#include "robotcontrol/bmcan_bus.hpp"
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <chrono>
#include <vector>


class ArmorSend : public rclcpp::Node {
public:
    ArmorSend() : Node("test_send_node") {
        RCLCPP_INFO(this->get_logger(), "%s节点已启动.", this->get_name());
        // 打开CAN通道
        BM_NotificationHandle result = canbus.open(channelhandle, "BM-CANFD-X1(5850)");
        if (result == nullptr) {
            RCLCPP_ERROR(this->get_logger(), "CAN设备打开失败!");
        } else {
            RCLCPP_INFO(this->get_logger(), "CAN设备已打开");
        }

        // 创建定时器，定期发送测试数据
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), // 100ms发送一次
            std::bind(&ArmorSend::timer_callback, this)
        );

        RCLCPP_INFO(this->get_logger(), "CAN测试节点已启动 - 将不断发送测试数据");
    }

    ~ArmorSend() {
        // 关闭CAN通道
        canbus.close(channelhandle);
        RCLCPP_INFO(this->get_logger(), "CAN设备已关闭");
    }

    // 发送测试数据
    void send_test_data() {
        
        uint8_t can_send_data[8] = {count, 1, 1, 1, 1, 1, 1, 1};

        RCLCPP_INFO(this->get_logger(), "发送测试数据: %d 1 1 1 1 1 1 1",count);
        try {
            canbus.can_send(channelhandle, cantx_id, can_send_data, 8);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "发送测试数据失败: %s", e.what());
        }
    }

private:
    BMCANTool canbus;
    BM_ChannelHandle channelhandle;
    // CAN通信相关ID
    const int cantx_id = 0x520; // 发送ID
    
    // 定时器
    rclcpp::TimerBase::SharedPtr timer_;
    uint8_t count =0;
    // 定时器回调函数
    void timer_callback() {
        send_test_data();
        count ++;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto armor_send_node = std::make_shared<ArmorSend>();
    rclcpp::spin(armor_send_node);
    rclcpp::shutdown();
    return 0;
}