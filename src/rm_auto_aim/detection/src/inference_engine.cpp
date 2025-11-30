#include "inference_engine.hpp"
#include <iostream>

namespace detection {

// 常量定义
constexpr int INFERENCE_THREADS = 14;

InferenceEngine::InferenceEngine(const std::string& model_path, int input_size, int inference_threads)
    : m_input_size(input_size)
{
    initialize_model(model_path, inference_threads);
}

InferenceEngine::~InferenceEngine()
{
    // 清理资源
}

void InferenceEngine::initialize_model(const std::string& model_path, int inference_threads)
{
    // 统一使用优化的配置参数
    ov::AnyMap config = {
        {ov::hint::performance_mode(ov::hint::PerformanceMode::THROUGHPUT)},  // 吞吐量模式，提高帧率
        {ov::inference_num_threads(inference_threads)},                       // 动态线程配置
        {ov::num_streams(1)},                                                 // 多流推理，提高并行性
        {ov::hint::enable_cpu_pinning(true)},                                 // 启用CPU固定，减少线程迁移开销
        {ov::hint::execution_mode(ov::hint::ExecutionMode::PERFORMANCE)},     // 性能执行模式
        {ov::log::level(ov::log::Level::WARNING)}                             // 减少日志输出，提高性能
    };

    auto network = m_core.read_model(model_path);
    m_compiled_model = m_core.compile_model(network, "CPU", config);
    m_infer_request = m_compiled_model.create_infer_request();
    m_input_port = m_compiled_model.input();
    m_output_ports = m_compiled_model.outputs();
    
    // 获取输出形状
    if (!m_output_ports.empty()) {
        m_output_shape = m_output_ports[0].get_shape();
    }
}

cv::Mat InferenceEngine::infer(const cv::Mat& input_image)
{
    // 归一化处理
    cv::Mat input_blob = cv::dnn::blobFromImage(
        input_image, 
        1 / 255.0
    );

    // 设置输入张量
    ov::Tensor input_tensor(m_input_port.get_element_type(), m_input_port.get_shape(), input_blob.data);
    m_infer_request.set_input_tensor(input_tensor);
    
    // 执行推理
    m_infer_request.infer();
    
    // 获取输出
    auto outputs = m_compiled_model.outputs();
    ov::Tensor output = m_infer_request.get_tensor(outputs[0]);
    ov::Shape output_shape = output.get_shape();
    
    // 转换为OpenCV矩阵
    cv::Mat output_buffer(output_shape[1], output_shape[2], CV_32F, output.data());
    
    return output_buffer;
}

}  // namespace detection
