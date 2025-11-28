#ifndef OPENVINO_PROFILER_HPP
#define OPENVINO_PROFILER_HPP

#include <openvino/openvino.hpp>
#include <string>
#include <vector>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <map>

namespace detection {

class OpenVINOProfiler {
public:
    struct LayerStats {
        std::string layer_name;
        double total_time_ms;
        int call_count;
        double avg_time_ms;
    };

    OpenVINOProfiler(const std::string& log_path = "openvino_profile.log");
    ~OpenVINOProfiler();

    // 收集单次推理的性能数据
    void collectProfilingData(const ov::InferRequest& infer_request);

    // 写入日志文件
    void writeLog();

private:
    std::string log_file_path_;
    std::map<std::string, LayerStats> layer_stats_;
    int total_inferences_;
    std::chrono::time_point<std::chrono::system_clock> start_time_;
    bool log_written_;
};

} // namespace detection

#endif // OPENVINO_PROFILER_HPP
