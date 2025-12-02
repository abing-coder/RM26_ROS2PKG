#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>
#include <chrono>
#include <iomanip>
#include "yolo_detection.hpp"

// 检查文件或目录是否存在
bool file_exists(const std::string& path) {
    struct stat buffer;
    return (stat(path.c_str(), &buffer) == 0);
}

// 获取文件名
std::string get_filename(const std::string& path) {
    size_t last_slash = path.find_last_of("/");
    if (last_slash != std::string::npos) {
        return path.substr(last_slash + 1);
    }
    return path;
}

// 获取文件扩展名
std::string get_extension(const std::string& path) {
    size_t last_dot = path.find_last_of(".");
    if (last_dot != std::string::npos) {
        return path.substr(last_dot);
    }
    return "";
}

int main(int argc, char** argv)
{
    // 获取当前工作目录
    char cwd[1024];
    bool got_cwd = getcwd(cwd, sizeof(cwd));
    if (got_cwd) {
        std::cout << "当前工作目录: " << cwd << std::endl;
    } else {
        std::cerr << "错误: 无法获取当前工作目录" << std::endl;
        return -1;
    }
    
    std::string current_path = cwd;
    
    // 设置模型路径和视频路径
    std::string model_path = current_path + "/src/rm_auto_aim/detection/model/IR_MODEL/new.xml";
    std::string video_dir = current_path + "/src/rm_auto_aim/detection/video/";
    
    // 检查视频目录是否存在
    if (!file_exists(video_dir)) {
        std::cerr << "错误: 视频目录不存在: " << video_dir << std::endl;
        return -1;
    }
    
    // 列出所有视频文件
    std::vector<std::string> video_files;
    DIR* dir = opendir(video_dir.c_str());
    if (dir) {
        struct dirent* entry;
        while ((entry = readdir(dir)) != nullptr) {
            std::string filename = entry->d_name;
            if (filename == "." || filename == "..") continue;
            
            std::string full_path = video_dir + filename;
            struct stat path_stat;
            if (stat(full_path.c_str(), &path_stat) == 0 && S_ISREG(path_stat.st_mode)) {
                std::string ext = get_extension(filename);
                if (ext == ".mp4" || ext == ".avi" || ext == ".mov") {
                    video_files.push_back(full_path);
                }
            }
        }
        closedir(dir);
    }
    
    if (video_files.empty()) {
        std::cerr << "错误: 在 " << video_dir << " 中没有找到视频文件" << std::endl;
        return -1;
    }
    
    std::cout << "找到 " << video_files.size() << " 个视频文件:" << std::endl;
    for (size_t i = 0; i < video_files.size(); ++i) {
        std::cout << "  " << i + 1 << ". " << get_filename(video_files[i]) << std::endl;
    }
    
    // 让用户选择视频文件
    int choice = 0;
    std::cout << "请选择要测试的视频文件 (1-" << video_files.size() << "): ";
    std::cin >> choice;
    
    if (choice < 1 || choice > static_cast<int>(video_files.size())) {
        std::cerr << "错误: 无效的选择" << std::endl;
        return -1;
    }
    
    std::string selected_video = video_files[choice - 1];
    std::cout << "选择的视频: " << selected_video << std::endl;
    
    // 检查模型文件是否存在
    if (!file_exists(model_path)) {
        std::cerr << "错误: 模型文件不存在: " << model_path << std::endl;
        return -1;
    }
    
    std::cout << "模型路径: " << model_path << std::endl;
    
    try {
        // 创建检测器实例
        std::cout << "初始化检测器..." << std::endl;
        detection::DetectionArmor detectionArmor(model_path, true, selected_video);

        std::cout << "开始目标检测..." << std::endl;
        std::cout << "按 'q' 键退出，按 'n' 键切换到下一个视频" << std::endl;

        // 帧率计算变量
        int frame_count = 0;
        auto start_time = std::chrono::high_resolution_clock::now();

        // 打开视频
        cv::VideoCapture cap(selected_video);
        if (!cap.isOpened()) {
            std::cerr << "错误: 无法打开视频文件" << std::endl;
            return -1;
        }

        cv::Mat frame;
        while (true) {
            // 读取帧
            cap >> frame;
            if (frame.empty()) {
                // 视频播放完毕，重置到开头循环播放
                cap.set(cv::CAP_PROP_POS_FRAMES, 0);
                continue;
            }

            // 执行检测
            detectionArmor.start_detection(frame);

            // 帧数计数
            frame_count++;

            // 每100帧计算一次平均帧率
            if (frame_count % 100 == 0) {
                auto end_time = std::chrono::high_resolution_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
                double avg_fps = 100.0 / (duration / 1000.0);

                std::cout << "已处理 " << frame_count << " 帧，最近100帧平均帧率: "
                          << std::fixed << std::setprecision(2) << avg_fps << " FPS" << std::endl;

                // 重置计时器
                start_time = std::chrono::high_resolution_clock::now();
            }

            // 显示结果
            #ifdef TEST_MODE
            detectionArmor.showImage();
            #endif

            // 检查按键
            int key = cv::waitKey(1);
            if (key == 'q' || key == 'Q') {
                std::cout << "用户退出" << std::endl;
                break;
            }
        }

        // 输出总体统计
        std::cout << "\n总共处理帧数: " << frame_count << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << std::endl;
        return -1;
    }
    
    return 0;
}
