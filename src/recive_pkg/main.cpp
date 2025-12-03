#include "recive_pkg/image_subscriber_node.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    
    // 修改5：模型路径 - 使用 OpenVINO IR 格式模型（与 test_aim.sh 一致）
    // 注意：此路径指向经过 OpenVINO PrePostProcessor 优化的 IR 模型，支持零拷贝推理
    // 如需更改模型，请同时更新 test_aim_node.cpp 保持一致
    const std::string model_path = "/home/ubuntu/桌面/Robomaster/RM26_ROS2PKG/src/rm_auto_aim/detection/model/IR_MODEL/new.xml";
   
    
    // 修改6：直接实例化节点
    auto node = std::make_shared<armor_detection::ImageSubscriber>(model_path);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}