#include "inference_engine.hpp"
#include <thread>
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
    // 实时推理优化：使用较少的线程数以减少调度开销
    int threads = inference_threads > 0 ? inference_threads : 4;
    const int hw_threads = static_cast<int>(std::thread::hardware_concurrency());
    if (hw_threads > 0) {
        threads = std::min(threads, hw_threads);
    }
    threads = std::max(1, threads);

    // 针对实时单帧推理的最优配置
    ov::AnyMap config = {
        {ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY)},  // 低延迟模式，适合实时推理
        {ov::hint::execution_mode(ov::hint::ExecutionMode::PERFORMANCE)},
        {ov::inference_num_threads(8)},  // 使用优化后的线程数
        {ov::num_streams(1)},  // 单流模式，避免多流开销
        {ov::hint::enable_cpu_pinning(true)},
        {ov::log::level(ov::log::Level::WARNING)}
    };

    auto network = m_core.read_model(model_path);
    m_compiled_model = m_core.compile_model(network, "CPU", config);
    m_infer_request = m_compiled_model.create_infer_request();
    m_input_port = m_compiled_model.input();
    m_output_ports = m_compiled_model.outputs();

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
