#include "recive_pkg/image_subscriber_node.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    
    // 修改5：通过参数传递模型路径
     const std::string model_path = "/home/ubuntu/桌面/RM26_ROS2PKG/src/rm_auto_aim/detection/model/new.onnx";
    // const bool debug_mode = true;  // 可从参数读取
   
    
    // 修改6：直接实例化节点
    auto node = std::make_shared<armor_detection::ImageSubscriber>(model_path);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}