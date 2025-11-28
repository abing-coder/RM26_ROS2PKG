#ifndef YOLO_DETECTION_HPP_
#define YOLO_DETECTION_HPP_

#include <iostream>
#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>
#include "timeCounter.hpp"
#include "traditional_detector.hpp"
#include "armor.hpp"
#include "openvino_profiler.hpp"

// 测试模式宏定义
#define TEST_MODE

// 枪管与相机偏移量常量（单位：米）
constexpr double GUN_CAM_DISTANCE_X = 0.0;
constexpr double GUN_CAM_DISTANCE_Y = 0.0;
constexpr double GUN_CAM_DISTANCE_Z = 0.0;

namespace detection
{

    /**
     * @brief 装甲板颜色枚举
     */
    enum class Color { RED, BLUE, NONE };

    /**
     * @brief 装甲板数据结构
     */
    struct ArmorData
    {
        cv::Point center_point;    ///< 目标中心点
        cv::Point optical_center;  ///< 光心点
        float delta_x;             ///< 目标中心点与光心点的x差值
        float delta_y;             ///< 目标中心点与光心点的y差值
        // float length;           ///< 长度
        // float width;            ///< 宽度

        cv::Point p1;  ///< 左上角点
        cv::Point p2;  ///< 右上角点  
        cv::Point p3;  ///< 右下角点
        cv::Point p4;  ///< 左下角点

        int ID = 0;    ///< 装甲板ID
        Color color;   ///< 装甲板颜色
    };

    /**
     * @brief 装甲板检测类
     */
    class DetectionArmor 
    {
    public:
        // 指定识别的颜色（共享变量，其他节点可能使用）
        static int detect_color;  ///< 0: 红色，1: 蓝色
        
        double fps;  ///< 帧率

        DetectionArmor() = default;  ///< 默认构造函数
        DetectionArmor(const DetectionArmor&) = delete;  ///< 禁止拷贝构造
        DetectionArmor& operator=(const DetectionArmor&) = delete;  ///< 禁止拷贝赋值
        DetectionArmor(DetectionArmor&&) = default;  ///< 默认移动构造
        DetectionArmor& operator=(DetectionArmor&&) = default;  ///< 默认移动赋值
        
        /**
         * @brief 构造函数
         * @param model_path 模型路径
         * @param if_count_time 是否计时
         * @param video_path 视频路径（可选）
         */
        DetectionArmor(std::string& model_path, bool if_count_time, std::string video_path = "");
        
        ~DetectionArmor();

        /**
         * @brief 在图像上绘制检测结果
         * @param image 输入图像
         * @param datas 装甲板数据
         */
        void drawObject(cv::Mat& image, std::vector<ArmorData>& datas);
        
        /**
         * @brief Sigmoid函数
         * @param x 输入值
         * @return Sigmoid计算结果
         */
        static double sigmoid(double x);

        /**
         * @brief 开始检测（使用视频源）
         */
        void start_detection();
        
        /**
         * @brief 开始检测（使用输入图像）
         * @param input_image 输入图像
         */
        void start_detection(const cv::Mat& input_image);
        
        /**
         * @brief 获取当前帧的装甲板数据
         * @return 装甲板数据引用
         */
        std::vector<ArmorData>& getdata();

        #ifdef TEST_MODE
        /**
         * @brief 格式化打印测试数据
         */
        void format_print_data_test();

        /**
         * @brief 显示图像（测试模式）
         */
        void showImage();
        #endif

        /**
         * @brief 运行检测主循环
         */
        void run();

    private:
        /// OpenVINO核心对象
        ov::Core m_core;
        /// 视频捕获对象
        cv::VideoCapture m_cap;
        /// 编译后的模型
        ov::CompiledModel m_compiled;
        /// 推理请求
        ov::InferRequest m_infer_request;
        /// 输入端口
        ov::Output<const ov::Node> m_input_port;

        /// 原始帧
        cv::Mat m_frame;
        /// 处理后的图像
        cv::Mat m_img;
        /// 输入blob
        cv::Mat m_input_blob;

        /// 计时器
        timeCounter m_counter = timeCounter("run a frame");
        /// 是否计时标志
        bool m_if_count_time = false;

        /// 当前帧的装甲板数据
        std::vector<ArmorData> m_armors_datas;

        /// 性能分析器
        OpenVINOProfiler m_profiler;

        /**
         * @brief 推理过程
         */
        void infer();

        /**
         * @brief 清理堆内存
         */
        void clearHeap();
    };

}  // namespace detection

#endif  // YOLO_DETECTION_HPP_
