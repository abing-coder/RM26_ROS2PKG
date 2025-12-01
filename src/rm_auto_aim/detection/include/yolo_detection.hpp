#ifndef __DETECTION_H__
#define __DETECTION_H__

#include <iostream>
#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>
#include <thread>
#include <atomic>
#include "timeCounter.hpp"
#include <shared_mutex>
#include "BYTETracker.h"
#include "traditional_detector.hpp"
#include "armor.hpp"

#define __TEST__
#define GUN_CAM_DISTANCE_X 0  
#define GUN_CAM_DISTANCE_Y 0
#define GUN_CAM_DISTANCE_Z 0

using namespace std;
using namespace cv;             
using namespace ov;
using namespace cv::dnn;


namespace detection
{

    enum class Color {RED, BLUE, NONE};

    typedef struct ArmorData
    {
        cv::Point center_point; // 目标中心点
        cv::Point optical_center;  // 光心点
        float delta_x;             // 目标中心点与光心点的x差值
        float delta_y;             // 目标中心点与光心点的y差值
        // float length;   //长度
        // float width;    //宽度
        int flag; //判断是否有装甲板

        cv::Point p1; // 左上
        cv::Point p2; // 右上
        cv::Point p3; // 右下
        cv::Point p4; // 左下

        int ID = 0;
        Color color;

    } ArmorData;

    class DetectionArmor 
    {

    // 一些基本的量
    Core core;
    VideoCapture cap;
    CompiledModel compiled;
    InferRequest infer_request;
    ov::Output<const ov::Node> input_port;

    Mat frame;
    Mat img;
    Mat input_blob;

    // 一个优雅又不太优雅的计时器
    timeCounter counter = timeCounter("run a frame");
    bool ifCountTime = false; // 是否计时


    // 当前帧的装甲板数据
    vector<ArmorData> armorsDatas; 

    std::vector<Object> detection_objects; // 要在容器里面存检测对象
    std::vector<STrack> tracks_objects;

    // 整个推理的线程
    thread infer_thread;

    // 一个用来保护识别到的装甲板的数据的锁 （不知道要不要用到）
    std::mutex _mtx;
    
    // 控制run线程的原子变量
    std::atomic<bool> isRunning{false};

    //数据就绪状态
    std::atomic<bool> data_ready{false};

    public:
        // 指定识别的颜色
        static int detect_color; // 0: 红色，1: 蓝色
        double fps;
        DetectionArmor() = default; //默认构造函数
        DetectionArmor(const DetectionArmor&) = delete; // 禁止拷贝
        DetectionArmor(string& model_path, bool ifCountTime, string video_path);
        DetectionArmor(string& model_path, bool ifCountTime);
        ~DetectionArmor();

        BYTETracker tracker = BYTETracker(10, 10); // 初始化BYTETracker

        void drawObject(Mat& image, vector<ArmorData>& datas);
        static double sigmoid(double x);

        void start_detection();
        void start_detection(const cv::Mat& input_image);
        

        // 外部线程可以开一个循环连续调用此成员函数以实时获取实时装甲板的检测结果
        vector<ArmorData>& getdata();

        void __TEST__ format_print_data_test();

        // 开启识别
        void run();

        void drawTracks(Mat& image);

        // 显示图像
        void __TEST__ showImage();

    private:
        void infer();

        void clearHeap();
    };

}  // namespace detection


#endif