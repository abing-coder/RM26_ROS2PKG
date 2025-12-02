#include "inference_engine.hpp"
#include <thread>
#include <iostream>

namespace detection {

// 常量定义
constexpr int INFERENCE_THREADS = 14;

InferenceEngine::InferenceEngine(const std::string& model_path, int input_size, int inference_threads)
    : m_input_size(input_size), m_model_path(model_path), m_inference_threads(inference_threads)
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
    m_inference_threads = threads;

    // 读取原始模型（延迟编译，等待知道输入尺寸后再使用PPP编译）
    m_network = m_core.read_model(model_path);
}

void InferenceEngine::recompile_with_ppp(int height, int width)
{
    using namespace ov::preprocess;

    // 克隆原始模型用于PPP处理
    auto model = m_network->clone();

    // 创建PrePostProcessor
    PrePostProcessor ppp(model);

    // 配置输入预处理：
    // 1. 输入张量属性：BGR u8 [1, H, W, 3] NHWC 布局
    // 2. 预处理步骤：BGR→RGB, u8→f32, /255 归一化
    // 3. 模型输入期望：RGB f32 [1, 3, 640, 640] NCHW 布局
    auto& input = ppp.input();

    // 设置输入张量属性（原始图像格式）
    input.tensor()
        .set_element_type(ov::element::u8)           // 输入为 uint8
        .set_layout("NHWC")                          // OpenCV Mat 是 HWC 格式
        .set_color_format(ov::preprocess::ColorFormat::BGR);  // OpenCV 默认 BGR

    // 预处理步骤（在推理前自动执行，零拷贝优化）
    // 输入已是 640x640，无需 resize
    input.preprocess()
        .convert_color(ov::preprocess::ColorFormat::RGB)  // BGR → RGB
        .convert_element_type(ov::element::f32)      // u8 → f32
        .scale(255.0f);                              // /255 归一化

    // 设置模型期望的输入格式
    input.model().set_layout("NCHW");

    // 构建带预处理的模型
    model = ppp.build();

    // 针对实时单帧推理的最优配置
    ov::AnyMap config = {
        {ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY)},
        {ov::hint::execution_mode(ov::hint::ExecutionMode::PERFORMANCE)},
        {ov::inference_num_threads(m_inference_threads)},
        {ov::num_streams(1)},
        {ov::hint::enable_cpu_pinning(true)},
        {ov::log::level(ov::log::Level::WARNING)}
    };

    // 编译模型
    m_compiled_model = m_core.compile_model(model, "CPU", config);
    m_infer_request = m_compiled_model.create_infer_request();
    m_input_port = m_compiled_model.input();
    m_output_ports = m_compiled_model.outputs();

    if (!m_output_ports.empty()) {
        m_output_shape = m_output_ports[0].get_shape();
    }

    // 更新当前输入尺寸
    m_current_input_height = height;
    m_current_input_width = width;

    std::cout << "[InferenceEngine] Compiled with PPP for input size: "
              << width << "x" << height << std::endl;
}

void InferenceEngine::set_input_image_size(int height, int width)
{
    if (m_current_input_height != height || m_current_input_width != width) {
        recompile_with_ppp(height, width);
    }
}


cv::Mat InferenceEngine::infer(const cv::Mat& input_image)
{
    // 检查输入尺寸是否变化，如果变化则重新编译模型
    if (m_current_input_height != input_image.rows ||
        m_current_input_width != input_image.cols) {
        recompile_with_ppp(input_image.rows, input_image.cols);
    }

    // 确保输入图像是连续的（零拷贝要求）
    cv::Mat continuous_input;
    if (input_image.isContinuous()) {
        continuous_input = input_image;
    } else {
        continuous_input = input_image.clone();
    }

    // 零拷贝：直接使用 OpenCV Mat 的数据指针创建输入张量
    // PPP 会自动处理 BGR→RGB, u8→f32, /255 归一化
    ov::Tensor input_tensor(
        ov::element::u8,
        {1, static_cast<size_t>(continuous_input.rows),
         static_cast<size_t>(continuous_input.cols), 3},
        continuous_input.data
    );

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
