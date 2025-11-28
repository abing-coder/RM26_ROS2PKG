#ifndef __PERFORMANCE_LOGGER_HPP__
#define __PERFORMANCE_LOGGER_HPP__

#include <iostream>
#include <fstream>
#include <chrono>
#include <string>
#include <mutex>

class PerformanceLogger {
public:
    static PerformanceLogger& getInstance() {
        static PerformanceLogger instance;
        return instance;
    }
    
    // 记录resize时间
    void logResizeTime(double time_ms) {
        std::lock_guard<std::mutex> lock(mutex_);
        resize_time_ = time_ms;
    }
    
    // 记录模型推理时间
    void logInferenceTime(double time_ms) {
        std::lock_guard<std::mutex> lock(mutex_);
        inference_time_ = time_ms;
    }
    
    // 记录传统opencv处理时间
    void logTraditionalProcessTime(double time_ms) {
        std::lock_guard<std::mutex> lock(mutex_);
        traditional_process_time_ = time_ms;
    }
    
    // 写入一帧的完整时间记录到日志文件
    void writeFrameLog() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!log_file_.is_open()) {
            log_file_.open("cost.log", std::ios::app);
        }
        
        if (log_file_.is_open()) {
            auto now = std::chrono::high_resolution_clock::now();
            auto time_since_epoch = now.time_since_epoch();
            auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(time_since_epoch);
            
            log_file_ << "Timestamp: " << microseconds.count() 
                      << ", Resize: " << resize_time_ << " ms"
                      << ", Inference: " << inference_time_ << " ms"
                      << ", TraditionalProcess: " << traditional_process_time_ << " ms"
                      << ", Total: " << (resize_time_ + inference_time_ + traditional_process_time_) << " ms"
                      << std::endl;
        }
    }
    
    // 重置时间记录
    void reset() {
        std::lock_guard<std::mutex> lock(mutex_);
        resize_time_ = 0.0;
        inference_time_ = 0.0;
        traditional_process_time_ = 0.0;
    }

private:
    PerformanceLogger() : resize_time_(0.0), inference_time_(0.0), traditional_process_time_(0.0) {
        log_file_.open("cost.log", std::ios::app);
    }
    
    ~PerformanceLogger() {
        if (log_file_.is_open()) {
            log_file_.close();
        }
    }
    
    // 禁止拷贝构造和赋值
    PerformanceLogger(const PerformanceLogger&) = delete;
    PerformanceLogger& operator=(const PerformanceLogger&) = delete;
    
    std::ofstream log_file_;
    double resize_time_;
    double inference_time_;
    double traditional_process_time_;
    std::mutex mutex_;
};

#endif // __PERFORMANCE_LOGGER_HPP__
