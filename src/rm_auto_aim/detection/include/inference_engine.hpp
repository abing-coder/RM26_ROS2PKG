#ifndef INFERENCE_ENGINE_HPP_
#define INFERENCE_ENGINE_HPP_

#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>
#include <openvino/preprocess/pre_post_process.hpp>
#include <string>
#include <vector>

namespace detection
{

    /**
     * @brief 推理引擎类，封装OpenVINO相关操作
     */
    class InferenceEngine 
    {
    public:
        InferenceEngine() = default;
        InferenceEngine(const InferenceEngine&) = delete;
        InferenceEngine& operator=(const InferenceEngine&) = delete;
        InferenceEngine(InferenceEngine&&) = default;
        InferenceEngine& operator=(InferenceEngine&&) = default;
        
        /**
         * @brief 构造函数
         * @param model_path 模型路径
         * @param input_size 输入尺寸
         * @param inference_threads 推理线程数
         */
        InferenceEngine(const std::string& model_path, int input_size = 640, int inference_threads = 14);
        
        ~InferenceEngine();

        /**
         * @brief 执行推理（接受任意尺寸图像，内部通过PPP处理resize）
         * @param input_image 输入图像（BGR格式，任意尺寸）
         * @return 推理输出矩阵
         */
        cv::Mat infer(const cv::Mat& input_image);

        /**
         * @brief 设置输入图像尺寸（当输入尺寸变化时需要重新编译模型）
         * @param height 图像高度
         * @param width 图像宽度
         */
        void set_input_image_size(int height, int width);

        /**
         * @brief 获取输入尺寸
         * @return 输入尺寸
         */
        int get_input_size() const { return m_input_size; }

        /**
         * @brief 获取模型输出形状
         * @return 输出形状
         */
        ov::Shape get_output_shape() const { return m_output_shape; }

    private:
        /// OpenVINO核心对象
        ov::Core m_core;
        /// 编译后的模型
        ov::CompiledModel m_compiled_model;
        /// 推理请求
        ov::InferRequest m_infer_request;
        /// 输入端口
        ov::Output<const ov::Node> m_input_port;
        /// 输出端口
        std::vector<ov::Output<const ov::Node>> m_output_ports;

        /// 输入尺寸（模型期望的尺寸）
        int m_input_size;
        /// 输出形状
        ov::Shape m_output_shape;
        /// 当前输入图像尺寸
        int m_current_input_height = 0;
        int m_current_input_width = 0;
        /// 模型路径（用于重新编译）
        std::string m_model_path;
        /// 推理线程数
        int m_inference_threads;
        /// 原始模型（未经PPP处理）
        std::shared_ptr<ov::Model> m_network;

        /**
         * @brief 初始化模型
         * @param model_path 模型路径
         * @param inference_threads 推理线程数
         */
        void initialize_model(const std::string& model_path, int inference_threads);

        /**
         * @brief 使用PPP重新编译模型（当输入尺寸变化时调用）
         * @param height 输入图像高度
         * @param width 输入图像宽度
         */
        void recompile_with_ppp(int height, int width);
    };

}  // namespace detection

#endif  // INFERENCE_ENGINE_HPP_
