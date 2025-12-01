#include "MvCameraControl.h"
// ROS
#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <pthread.h>
#include <sched.h>

namespace hik_camera
{
class HikCameraNode : public rclcpp::Node
{
public:
  explicit HikCameraNode(const rclcpp::NodeOptions & options) : Node("hik_camera", options)
  {
    RCLCPP_INFO(this->get_logger(), "Starting HikCameraNode!");

    // 初始化相机设备
    MV_CC_DEVICE_INFO_LIST device_list;
    nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
    
    if (MV_OK != nRet) {
      RCLCPP_ERROR(this->get_logger(), "Enum devices failed! nRet: [%x]", nRet);
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Found camera count = %d", device_list.nDeviceNum);

    // 等待相机连接
    while (device_list.nDeviceNum == 0 && rclcpp::ok()) {
      RCLCPP_WARN(this->get_logger(), "No camera found! Retrying...");
      std::this_thread::sleep_for(std::chrono::seconds(1));
      nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
    }

    if (!rclcpp::ok()) {
      return;
    }

    // 创建相机句柄
    nRet = MV_CC_CreateHandle(&camera_handle_, device_list.pDeviceInfo[0]);
    if (MV_OK != nRet) {
      RCLCPP_ERROR(this->get_logger(), "Create handle failed! nRet: [%x]", nRet);
      return;
    }

    // 打开设备
    nRet = MV_CC_OpenDevice(camera_handle_);
    if (MV_OK != nRet) {
      RCLCPP_ERROR(this->get_logger(), "Open device failed! nRet: [%x]", nRet);
      return;
    }

    // 设置相机参数
    declareParameters();

    // 获取图像信息
    nRet = MV_CC_GetImageInfo(camera_handle_, &img_info_);
    if (MV_OK != nRet) {
      RCLCPP_ERROR(this->get_logger(), "Get image info failed! nRet: [%x]", nRet);
      return;
    }

    // 初始化图像消息
    image_msg_.encoding = "rgb8";
    image_msg_.header.frame_id = "camera_optical_frame";

    // 初始化像素转换参数
    convert_param_.nWidth = img_info_.nWidthValue;
    convert_param_.nHeight = img_info_.nHeightValue;
    convert_param_.enDstPixelType = PixelType_Gvsp_RGB8_Packed;

    // 设置图像节点数量（缓冲区）- 增加缓冲区以提高性能
    int nBufferNum = this->declare_parameter("buffer_num", 8);
    nRet = MV_CC_SetImageNodeNum(camera_handle_, nBufferNum);
    if (MV_OK != nRet) {
      RCLCPP_WARN(this->get_logger(), "Set image node num failed! nRet: [%x]", nRet);
    } else {
      RCLCPP_INFO(this->get_logger(), "Image buffer num set to: %d", nBufferNum);
    }

    // 创建图像发布者 - 使用最佳性能的 QoS 配置
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
    qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);  // BestEffort 更适合实时图像流
    qos.durability(rclcpp::DurabilityPolicy::Volatile);
    qos.avoid_ros_namespace_conventions(false);
    qos.keep_last(1);  // 只保留最新一帧，减少内存占用

    // 创建发布者
    camera_pub_ = image_transport::create_camera_publisher(this, "/image_raw", qos.get_rmw_qos_profile());

    // 加载相机标定信息
    camera_name_ = this->declare_parameter("camera_name", "hik_camera");
    camera_info_manager_ = std::make_unique<camera_info_manager::CameraInfoManager>(this, camera_name_);
    
    auto camera_info_url = this->declare_parameter("camera_info_url", "");
    if (!camera_info_url.empty() && camera_info_manager_->validateURL(camera_info_url)) {
      camera_info_manager_->loadCameraInfo(camera_info_url);
      camera_info_msg_ = camera_info_manager_->getCameraInfo();
      RCLCPP_INFO(this->get_logger(), "Loaded camera info from: %s", camera_info_url.c_str());
    } else {
      RCLCPP_WARN(this->get_logger(), "Using default camera info");
      // 设置默认相机信息
      camera_info_msg_.height = img_info_.nHeightValue;
      camera_info_msg_.width = img_info_.nWidthValue;
      camera_info_msg_.header.frame_id = "camera_optical_frame";
    }

    // 开始采集图像
    nRet = MV_CC_StartGrabbing(camera_handle_);
    if (MV_OK != nRet) {
      RCLCPP_ERROR(this->get_logger(), "Start grabbing failed! nRet: [%x]", nRet);
      return;
    }

    // 参数回调
    params_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&HikCameraNode::parametersCallback, this, std::placeholders::_1));

    // 创建采集线程
    capture_thread_ = std::thread{[this]() -> void {
      RCLCPP_INFO(this->get_logger(), "Capture thread started!");

      MV_FRAME_OUT out_frame;

      while (rclcpp::ok()) {
        nRet = MV_CC_GetImageBuffer(camera_handle_, &out_frame, 1000);
        
        if (MV_OK == nRet) {
          // 准备转换参数
          convert_param_.pSrcData = out_frame.pBufAddr;
          convert_param_.nSrcDataLen = out_frame.stFrameInfo.nFrameLen;
          convert_param_.enSrcPixelType = out_frame.stFrameInfo.enPixelType;

          // 分配目标缓冲区
          size_t dst_buffer_size = out_frame.stFrameInfo.nWidth * out_frame.stFrameInfo.nHeight * 3;
          image_msg_.data.resize(dst_buffer_size);
          convert_param_.pDstBuffer = image_msg_.data.data();
          convert_param_.nDstBufferSize = dst_buffer_size;

          // 转换像素格式
          nRet = MV_CC_ConvertPixelType(camera_handle_, &convert_param_);
          if (MV_OK == nRet) {
            // 设置图像消息
            image_msg_.header.stamp = this->now();
            image_msg_.height = out_frame.stFrameInfo.nHeight;
            image_msg_.width = out_frame.stFrameInfo.nWidth;
            image_msg_.step = out_frame.stFrameInfo.nWidth * 3;

            // 设置相机信息消息
            camera_info_msg_.header = image_msg_.header;

            // 发布消息
            camera_pub_.publish(image_msg_, camera_info_msg_);
          } else {
            RCLCPP_WARN(this->get_logger(), "Pixel conversion failed! nRet: [%x]", nRet);
          }

          // 释放图像缓冲区
          MV_CC_FreeImageBuffer(camera_handle_, &out_frame);
          fail_count_ = 0;
        } else {
          RCLCPP_WARN(this->get_logger(), "Get image buffer failed! nRet: [%x]", nRet);
          fail_count_++;
          
          // 如果连续失败多次，尝试重新启动采集
          if (fail_count_ > 5) {
            RCLCPP_ERROR(this->get_logger(), "Multiple failures, restarting grabber...");
            MV_CC_StopGrabbing(camera_handle_);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            nRet = MV_CC_StartGrabbing(camera_handle_);
            if (MV_OK == nRet) {
              fail_count_ = 0;
              RCLCPP_INFO(this->get_logger(), "Grabber restarted successfully");
            } else {
              RCLCPP_FATAL(this->get_logger(), "Failed to restart grabber! nRet: [%x]", nRet);
              break;
            }
          }
        }
      }
    }};
  }

  ~HikCameraNode() override
  {
    RCLCPP_INFO(this->get_logger(), "Shutting down HikCameraNode...");
    
    // 停止采集线程
    if (capture_thread_.joinable()) {
      capture_thread_.join();
    }
    
    // 关闭相机
    if (camera_handle_) {
      MV_CC_StopGrabbing(camera_handle_);
      MV_CC_CloseDevice(camera_handle_);
      MV_CC_DestroyHandle(&camera_handle_);
    }
    
    RCLCPP_INFO(this->get_logger(), "HikCameraNode destroyed!");
  }

private:
  void declareParameters()
  {
    // 设置分辨率
    int width = this->declare_parameter("width", 640);
    int height = this->declare_parameter("height", 640);
    
    nRet = MV_CC_SetIntValue(camera_handle_, "Width", width);
    if (MV_OK != nRet) {
      RCLCPP_WARN(this->get_logger(), "Set width failed! nRet: [%x]", nRet);
    }
    
    nRet = MV_CC_SetIntValue(camera_handle_, "Height", height);
    if (MV_OK != nRet) {
      RCLCPP_WARN(this->get_logger(), "Set height failed! nRet: [%x]", nRet);
    }
    
    RCLCPP_INFO(this->get_logger(), "Resolution set to: %dx%d", width, height);

    // 设置曝光时间
    MVCC_FLOATVALUE exposure_range;
    nRet = MV_CC_GetFloatValue(camera_handle_, "ExposureTime", &exposure_range);
    if (MV_OK == nRet) {
        double exposure_time = this->declare_parameter("exposure_time", 5000.0);
        nRet = MV_CC_SetFloatValue(camera_handle_, "ExposureTime", exposure_time);
        if (MV_OK == nRet) {
            RCLCPP_INFO(this->get_logger(), "Exposure time set to: %f us", exposure_time);
        } else {
            RCLCPP_WARN(this->get_logger(), "Set exposure time failed! nRet: [%x]", nRet);
        }
    }

    // 设置增益
    MVCC_FLOATVALUE gain_range;
    nRet = MV_CC_GetFloatValue(camera_handle_, "Gain", &gain_range);
    if (MV_OK == nRet) {
      double gain = this->declare_parameter("gain", static_cast<double>(gain_range.fCurValue));
      nRet = MV_CC_SetFloatValue(camera_handle_, "Gain", gain);
      if (MV_OK == nRet) {
        RCLCPP_INFO(this->get_logger(), "Gain set to: %f", gain);
      } else {
        RCLCPP_WARN(this->get_logger(), "Set gain failed! nRet: [%x]", nRet);
      }
    }

    // 设置帧率
    MVCC_FLOATVALUE frame_rate_range;
    nRet = MV_CC_GetFloatValue(camera_handle_, "ResultingFrameRate", &frame_rate_range);
    if (MV_OK == nRet) {
      double frame_rate = this->declare_parameter("frame_rate", 
        static_cast<double>(frame_rate_range.fCurValue));
      nRet = MV_CC_SetFloatValue(camera_handle_, "ResultingFrameRate", frame_rate);
      if (MV_OK == nRet) {
        RCLCPP_INFO(this->get_logger(), "Frame rate set to: %f fps", frame_rate);
      } else {
        RCLCPP_WARN(this->get_logger(), "Set frame rate failed! nRet: [%x]", nRet);
      }
    }
  }

  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    
    for (const auto & param : parameters) {
      if (param.get_name() == "exposure_time") {
        nRet = MV_CC_SetFloatValue(camera_handle_, "ExposureTime", param.as_double());
        if (MV_OK != nRet) {
          result.successful = false;
          result.reason = "Failed to set exposure time, nRet: " + std::to_string(nRet);
        }
      } else if (param.get_name() == "gain") {
        nRet = MV_CC_SetFloatValue(camera_handle_, "Gain", param.as_double());
        if (MV_OK != nRet) {
          result.successful = false;
          result.reason = "Failed to set gain, nRet: " + std::to_string(nRet);
        }
      } else if (param.get_name() == "frame_rate") {
        nRet = MV_CC_SetFloatValue(camera_handle_, "ResultingFrameRate", param.as_double());
        if (MV_OK != nRet) {
          result.successful = false;
          result.reason = "Failed to set frame rate, nRet: " + std::to_string(nRet);
        }
      } else {
        result.successful = false;
        result.reason = "Unknown parameter: " + param.get_name();
      }
    }
    
    return result;
  }

  sensor_msgs::msg::Image image_msg_;
  image_transport::CameraPublisher camera_pub_;

  int nRet = MV_OK;
  void * camera_handle_ = nullptr;
  MV_IMAGE_BASIC_INFO img_info_;
  MV_CC_PIXEL_CONVERT_PARAM convert_param_;

  std::string camera_name_;
  std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
  sensor_msgs::msg::CameraInfo camera_info_msg_;

  int fail_count_ = 0;
  std::thread capture_thread_;
  OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;
};
}  // namespace hik_camera

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(hik_camera::HikCameraNode)