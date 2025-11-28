#include "openvino_profiler.hpp"
#include <iostream>
#include <csignal>
#include <algorithm>

namespace detection {

// 全局指针，用于信号处理
static OpenVINOProfiler* g_profiler_instance = nullptr;

// 信号处理函数
void signal_handler(int signal) {
    if (g_profiler_instance != nullptr) {
        std::cout << "\n捕获到退出信号，正在写入性能日志..." << std::endl;
        g_profiler_instance->writeLog();
    }
    std::exit(signal);
}

OpenVINOProfiler::OpenVINOProfiler(const std::string& log_path)
    : log_file_path_(log_path), total_inferences_(0), log_written_(false) {
    start_time_ = std::chrono::system_clock::now();

    // 注册信号处理器，捕获 Ctrl+C
    g_profiler_instance = this;
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);
}

OpenVINOProfiler::~OpenVINOProfiler() {
    // 析构时自动写入日志
    if (g_profiler_instance == this) {
        writeLog();
        g_profiler_instance = nullptr;
    }
}

void OpenVINOProfiler::collectProfilingData(const ov::InferRequest& infer_request) {
    total_inferences_++;

    // 获取性能分析数据
    auto perf_counts = infer_request.get_profiling_info();

    for (const auto& layer : perf_counts) {
        std::string layer_name = layer.node_name;
        double layer_time_ms = layer.real_time.count() / 1000.0; // 转换为毫秒

        // 累积统计数据
        if (layer_stats_.find(layer_name) == layer_stats_.end()) {
            layer_stats_[layer_name] = {layer_name, 0.0, 0, 0.0};
        }

        layer_stats_[layer_name].total_time_ms += layer_time_ms;
        layer_stats_[layer_name].call_count++;
    }
}

void OpenVINOProfiler::writeLog() {
    // 防止重复写入
    if (log_written_) {
        return;
    }

    if (total_inferences_ == 0) {
        std::cout << "没有收集到性能数据，跳过日志写入" << std::endl;
        return;
    }

    log_written_ = true;

    // 计算平均时间
    for (auto& pair : layer_stats_) {
        pair.second.avg_time_ms = pair.second.total_time_ms / pair.second.call_count;
    }

    // 写入日志文件
    std::ofstream log_file(log_file_path_);
    if (!log_file.is_open()) {
        std::cerr << "无法打开日志文件: " << log_file_path_ << std::endl;
        return;
    }

    auto end_time = std::chrono::system_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time_);
    double duration_seconds = duration.count() / 1000.0;

    // 计算平均帧率
    double avg_fps = (duration_seconds > 0) ? (total_inferences_ / duration_seconds) : 0.0;

    log_file << "=================================================\n";
    log_file << "       OpenVINO 性能分析报告\n";
    log_file << "=================================================\n\n";
    log_file << "总推理次数: " << total_inferences_ << "\n";
    log_file << "运行时长: " << std::fixed << std::setprecision(2) << duration_seconds << " 秒\n";
    log_file << "平均帧率: " << std::fixed << std::setprecision(2) << avg_fps << " FPS\n\n";

    log_file << std::string(80, '-') << "\n";
    log_file << std::left << std::setw(50) << "层名称"
             << std::right << std::setw(12) << "总耗时(ms)"
             << std::setw(10) << "调用次数"
             << std::setw(12) << "平均耗时(ms)" << "\n";
    log_file << std::string(80, '-') << "\n";

    // 按平均耗时排序
    std::vector<LayerStats> sorted_stats;
    for (const auto& pair : layer_stats_) {
        sorted_stats.push_back(pair.second);
    }
    std::sort(sorted_stats.begin(), sorted_stats.end(),
              [](const LayerStats& a, const LayerStats& b) {
                  return a.avg_time_ms > b.avg_time_ms;
              });

    double total_time = 0.0;
    for (const auto& stats : sorted_stats) {
        log_file << std::left << std::setw(50) << stats.layer_name
                 << std::right << std::fixed << std::setprecision(2)
                 << std::setw(12) << stats.total_time_ms
                 << std::setw(10) << stats.call_count
                 << std::setw(12) << stats.avg_time_ms << "\n";
        total_time += stats.avg_time_ms;
    }

    log_file << std::string(80, '-') << "\n";
    log_file << std::left << std::setw(50) << "单次推理总耗时"
             << std::right << std::setw(12) << ""
             << std::setw(10) << ""
             << std::setw(12) << std::fixed << std::setprecision(2) << total_time << "\n";

    // 计算每帧推理耗时平均
    double avg_inference_time = total_time;
    log_file << std::left << std::setw(50) << "每帧推理耗时平均"
             << std::right << std::setw(12) << ""
             << std::setw(10) << ""
             << std::setw(12) << std::fixed << std::setprecision(2) << avg_inference_time << " ms\n";

    log_file << "=================================================\n";

    log_file.close();
}

} // namespace detection
