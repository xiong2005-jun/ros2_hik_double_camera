# ros2_hik_camera

A ROS2 packge for Hikvision USB3.0 industrial camera

## Usage

```
ros2 launch hik_camera hik_camera.launch.py
```

## Params

- exposure_time
- gain



______________________________

# 1. 安装标定工具
sudo apt install ros-humble-camera-calibration


source install/setup.bash

# 2. 打开相机
ros2 launch hik_camera hik_camera.launch.py

# 3. 在新终端中启动标定器（使用正确的参数格式）
ros2 run camera_calibration cameracalibrator --size 6x9 --square 0.024 image:=/image_raw camera:=/camera





----------------------------------------------------------------

image_width: 1440
image_height: 1080
camera_name: hik_camera
camera_matrix:
  rows: 3
  cols: 3
  data: [1794.781576, 0.000000, 689.470511, 0.000000, 1798.539007, 561.693360, 0.000000, 0.000000, 1.000000]
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [-0.074415, 0.103957, -0.000629, -0.002827, 0.000000]
rectification_matrix:
  rows: 3
  cols: 3
  data: [1.000000, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000, 0.000000, 0.000000, 1.000000]
projection_matrix:
  rows: 3
  cols: 4
  data: [1777.324582, 0.000000, 686.542206, 0.000000, 0.000000, 1788.115914, 561.594873, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000]
  

##//======一些坑=======
ldconfig -p | grep MvCamera 无输出
海康驱动装了，但是 没有写入系统库路径
echo "/opt/MVS/lib/64" | sudo tee /etc/ld.so.conf.d/hikrobot.conf
sudo ldconfig


####//======历史版本============//####


#include "MvCameraControl.h"
// ROS
#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace hik_camera
{
class HikCameraNode : public rclcpp::Node
{
public:
  explicit HikCameraNode(const rclcpp::NodeOptions & options) : Node("hik_camera", options)
  {
    RCLCPP_INFO(this->get_logger(), "Starting HikCameraNode!");

    MV_CC_DEVICE_INFO_LIST device_list;
    // enum device
    nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
    RCLCPP_INFO(this->get_logger(), "Found camera count = %d", device_list.nDeviceNum);

    while (device_list.nDeviceNum == 0 && rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "No camera found!");
      RCLCPP_INFO(this->get_logger(), "Enum state: [%x]", nRet);
      std::this_thread::sleep_for(std::chrono::seconds(1));
      nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
    }

    MV_CC_CreateHandle(&camera_handle_, device_list.pDeviceInfo[0]);

    MV_CC_OpenDevice(camera_handle_);

    // Get camera infomation
    MV_CC_GetImageInfo(camera_handle_, &img_info_);
    image_msg_.data.reserve(img_info_.nHeightMax * img_info_.nWidthMax * 3);

    // Init convert param
    convert_param_.nWidth = img_info_.nWidthValue;
    convert_param_.nHeight = img_info_.nHeightValue;
    convert_param_.enDstPixelType = PixelType_Gvsp_RGB8_Packed;

    bool use_sensor_data_qos = this->declare_parameter("use_sensor_data_qos", true);
    auto qos = use_sensor_data_qos ? rmw_qos_profile_sensor_data : rmw_qos_profile_default;
    camera_pub_ = image_transport::create_camera_publisher(this, "image_raw", qos);

    declareParameters();

    MV_CC_StartGrabbing(camera_handle_);

    // Load camera info
    camera_name_ = this->declare_parameter("camera_name", "narrow_stereo");
    camera_info_manager_ =
      std::make_unique<camera_info_manager::CameraInfoManager>(this, camera_name_);
    auto camera_info_url =
      this->declare_parameter("camera_info_url", "package://hik_camera/config/camera_info.yaml");
    if (camera_info_manager_->validateURL(camera_info_url)) {
      camera_info_manager_->loadCameraInfo(camera_info_url);
      camera_info_msg_ = camera_info_manager_->getCameraInfo();
    } else {
      RCLCPP_WARN(this->get_logger(), "Invalid camera info URL: %s", camera_info_url.c_str());
    }

    params_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&HikCameraNode::parametersCallback, this, std::placeholders::_1));

    capture_thread_ = std::thread{[this]() -> void {
      MV_FRAME_OUT out_frame;

      RCLCPP_INFO(this->get_logger(), "Publishing image!");

      image_msg_.header.frame_id = "camera_optical_frame";
      image_msg_.encoding = "rgb8";

      while (rclcpp::ok()) {
        nRet = MV_CC_GetImageBuffer(camera_handle_, &out_frame, 1000);
        if (MV_OK == nRet) {
          convert_param_.pDstBuffer = image_msg_.data.data();
          convert_param_.nDstBufferSize = image_msg_.data.size();
          convert_param_.pSrcData = out_frame.pBufAddr;
          convert_param_.nSrcDataLen = out_frame.stFrameInfo.nFrameLen;
          convert_param_.enSrcPixelType = out_frame.stFrameInfo.enPixelType;

          MV_CC_ConvertPixelType(camera_handle_, &convert_param_);

          image_msg_.header.stamp = this->now();
          image_msg_.height = out_frame.stFrameInfo.nHeight;
          image_msg_.width = out_frame.stFrameInfo.nWidth;
          image_msg_.step = out_frame.stFrameInfo.nWidth * 3;
          image_msg_.data.resize(image_msg_.width * image_msg_.height * 3);

          camera_info_msg_.header = image_msg_.header;
          camera_pub_.publish(image_msg_, camera_info_msg_);

          MV_CC_FreeImageBuffer(camera_handle_, &out_frame);
          fail_conut_ = 0;
        } else {
          RCLCPP_WARN(this->get_logger(), "Get buffer failed! nRet: [%x]", nRet);
          MV_CC_StopGrabbing(camera_handle_);
          MV_CC_StartGrabbing(camera_handle_);
          fail_conut_++;
        }

        if (fail_conut_ > 5) {
          RCLCPP_FATAL(this->get_logger(), "Camera failed!");
          rclcpp::shutdown();
        }
      }
    }};
  }

  ~HikCameraNode() override
  {
    if (capture_thread_.joinable()) {
      capture_thread_.join();
    }
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
    rcl_interfaces::msg::ParameterDescriptor param_desc;
    MVCC_FLOATVALUE f_value;
    param_desc.integer_range.resize(1);
    param_desc.integer_range[0].step = 1;
    // Exposure time
    param_desc.description = "Exposure time in microseconds";
    MV_CC_GetFloatValue(camera_handle_, "ExposureTime", &f_value);
    param_desc.integer_range[0].from_value = f_value.fMin;
    param_desc.integer_range[0].to_value = f_value.fMax;
    double exposure_time = this->declare_parameter("exposure_time", 5000, param_desc);
    MV_CC_SetFloatValue(camera_handle_, "ExposureTime", exposure_time);
    RCLCPP_INFO(this->get_logger(), "Exposure time: %f", exposure_time);

    // Gain
    param_desc.description = "Gain";
    MV_CC_GetFloatValue(camera_handle_, "Gain", &f_value);
    param_desc.integer_range[0].from_value = f_value.fMin;
    param_desc.integer_range[0].to_value = f_value.fMax;
    double gain = this->declare_parameter("gain", f_value.fCurValue, param_desc);
    MV_CC_SetFloatValue(camera_handle_, "Gain", gain);
    RCLCPP_INFO(this->get_logger(), "Gain: %f", gain);
  }

  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto & param : parameters) {
      if (param.get_name() == "exposure_time") {
        int status = MV_CC_SetFloatValue(camera_handle_, "ExposureTime", param.as_int());
        if (MV_OK != status) {
          result.successful = false;
          result.reason = "Failed to set exposure time, status = " + std::to_string(status);
        }
      } else if (param.get_name() == "gain") {
        int status = MV_CC_SetFloatValue(camera_handle_, "Gain", param.as_double());
        if (MV_OK != status) {
          result.successful = false;
          result.reason = "Failed to set gain, status = " + std::to_string(status);
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
  void * camera_handle_;
  MV_IMAGE_BASIC_INFO img_info_;

  MV_CC_PIXEL_CONVERT_PARAM convert_param_;

  std::string camera_name_;
  std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
  sensor_msgs::msg::CameraInfo camera_info_msg_;

  int fail_conut_ = 0;
  std::thread capture_thread_;

  OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;
};
}  // namespace hik_camera

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(hik_camera::HikCameraNode)


*/

//=========xiong：以上为君佬原版==============



/*
#include "MvCameraControl.h"
// ROS
#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <unistd.h>
#include <cstdlib>

namespace hik_camera
{
class HikCameraNode : public rclcpp::Node
{
public:
  explicit HikCameraNode(const rclcpp::NodeOptions & options) : Node("hik_camera", options)
  {
    RCLCPP_INFO(this->get_logger(), "Starting HikCameraNode!");

    // 连接相机（如果失败会无限重试）
    if (!connectCameraWithRetry()) {
      RCLCPP_FATAL(this->get_logger(), "Camera connection failed and no retry possible, exiting.");
      rclcpp::shutdown();
      return;
    }

    // 正常初始化其他部分...
    bool use_sensor_data_qos = this->declare_parameter("use_sensor_data_qos", true);
    auto qos = use_sensor_data_qos ? rmw_qos_profile_sensor_data : rmw_qos_profile_default;
    camera_pub_ = image_transport::create_camera_publisher(this, "image_raw", qos);

    camera_name_ = this->declare_parameter("camera_name", "narrow_stereo");
    camera_info_manager_ = std::make_unique<camera_info_manager::CameraInfoManager>(this, camera_name_);
    auto camera_info_url = this->declare_parameter("camera_info_url", "package://hik_camera/config/camera_info.yaml");
    if (camera_info_manager_->validateURL(camera_info_url)) {
      camera_info_manager_->loadCameraInfo(camera_info_url);
      camera_info_msg_ = camera_info_manager_->getCameraInfo();
    } else {
      RCLCPP_WARN(this->get_logger(), "Invalid camera info URL: %s", camera_info_url.c_str());
    }

    params_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&HikCameraNode::parametersCallback, this, std::placeholders::_1));

    capture_thread_ = std::thread{[this]() -> void {
      MV_FRAME_OUT out_frame;
      RCLCPP_INFO(this->get_logger(), "Publishing image!");

      image_msg_.header.frame_id = "camera_optical_frame";
      image_msg_.encoding = "rgb8";

      while (rclcpp::ok()) {
        int ret = MV_CC_GetImageBuffer(camera_handle_, &out_frame, 1000);
        if (MV_OK == ret) {
          // 成功获取图像
          convert_param_.pDstBuffer = image_msg_.data.data();
          convert_param_.nDstBufferSize = image_msg_.data.size();
          convert_param_.pSrcData = out_frame.pBufAddr;
          convert_param_.nSrcDataLen = out_frame.stFrameInfo.nFrameLen;
          convert_param_.enSrcPixelType = out_frame.stFrameInfo.enPixelType;

          MV_CC_ConvertPixelType(camera_handle_, &convert_param_);

          image_msg_.header.stamp = this->now();
          image_msg_.height = out_frame.stFrameInfo.nHeight;
          image_msg_.width = out_frame.stFrameInfo.nWidth;
          image_msg_.step = out_frame.stFrameInfo.nWidth * 3;
          image_msg_.data.resize(image_msg_.width * image_msg_.height * 3);

          camera_info_msg_.header = image_msg_.header;
          camera_pub_.publish(image_msg_, camera_info_msg_);

          MV_CC_FreeImageBuffer(camera_handle_, &out_frame);
          fail_count_ = 0;  // 成功时重置失败计数
        } else {
          RCLCPP_WARN(this->get_logger(), "Get buffer failed! nRet: [%x]", ret);
          fail_count_++;
          if (fail_count_ >= 5) {
            RCLCPP_FATAL(this->get_logger(), "Camera streaming failed repeatedly, attempting to reconnect...");
            // 尝试重连
            if (reconnectCamera()) {
              RCLCPP_INFO(this->get_logger(), "Camera reconnected successfully.");
              fail_count_ = 0;
            } else {
              RCLCPP_ERROR(this->get_logger(), "Camera reconnection failed, will retry again after 2milliseconds .");
              // 继续循环，下一次循环会再次尝试重连（因为 camera_handle_ 可能无效，需要重新初始化）
            }
          }
        }
      }
    }};
  }

  ~HikCameraNode() override
  {
    if (capture_thread_.joinable()) {
      capture_thread_.join();
    }
    disconnectCamera();
    RCLCPP_INFO(this->get_logger(), "HikCameraNode destroyed!");
  }

private:
  // 连接相机，如果失败会一直重试（无限等待）
  bool connectCameraWithRetry()
  {
    while (rclcpp::ok()) {
      if (connectCamera()) {
        return true;
      }
      RCLCPP_ERROR(this->get_logger(), "Camera connection failed, retrying in 2 milliseconds");
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return false;
  }

  // 连接相机（单次尝试，不循环）
  bool connectCamera()
  {
    std::lock_guard<std::mutex> lock(camera_mutex_);

    MV_CC_DEVICE_INFO_LIST device_list;
    int ret = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
    if (ret != MV_OK) {
      RCLCPP_ERROR(this->get_logger(), "Enumeration failed, error: [%x]", ret);
      return false;
    }

    if (device_list.nDeviceNum == 0) {
      RCLCPP_ERROR(this->get_logger(), "No camera found");
      return false;
    }

    ret = MV_CC_CreateHandle(&camera_handle_, device_list.pDeviceInfo[0]);
    if (ret != MV_OK) {
      RCLCPP_ERROR(this->get_logger(), "Create handle failed, error: [%x]", ret);
      return false;
    }

    ret = MV_CC_OpenDevice(camera_handle_);
    if (ret != MV_OK) {
      RCLCPP_ERROR(this->get_logger(), "Open device failed, error: [%x]", ret);
      MV_CC_DestroyHandle(&camera_handle_);
      camera_handle_ = nullptr;
      return false;
    }

    ret = MV_CC_GetImageInfo(camera_handle_, &img_info_);
    if (ret != MV_OK) {
      RCLCPP_ERROR(this->get_logger(), "Get image info failed, error: [%x]", ret);
      MV_CC_CloseDevice(camera_handle_);
      MV_CC_DestroyHandle(&camera_handle_);
      camera_handle_ = nullptr;
      return false;
    }

    // 初始化转换参数
    convert_param_.nWidth = img_info_.nWidthValue;
    convert_param_.nHeight = img_info_.nHeightValue;
    convert_param_.enDstPixelType = PixelType_Gvsp_RGB8_Packed;
    image_msg_.data.reserve(img_info_.nHeightMax * img_info_.nWidthMax * 3);

    // 声明并设置相机参数
    declareParameters();

    ret = MV_CC_StartGrabbing(camera_handle_);
    if (ret != MV_OK) {
      RCLCPP_ERROR(this->get_logger(), "Start grabbing failed, error: [%x]", ret);
      MV_CC_CloseDevice(camera_handle_);
      MV_CC_DestroyHandle(&camera_handle_);
      camera_handle_ = nullptr;
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "Camera connected successfully");
    return true;
  }

  // 重连相机：断开现有连接，然后尝试重新连接（会循环重试直到成功）
  bool reconnectCamera()
  {
    RCLCPP_INFO(this->get_logger(), "Attempting to reconnect camera...");
    disconnectCamera();  // 释放当前资源
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    return connectCameraWithRetry();  // 一直重试直到成功
  }

  void disconnectCamera()
  {
    std::lock_guard<std::mutex> lock(camera_mutex_);
    if (camera_handle_) {
      MV_CC_StopGrabbing(camera_handle_);
      MV_CC_CloseDevice(camera_handle_);
      MV_CC_DestroyHandle(&camera_handle_);
      camera_handle_ = nullptr;
    }
  }

  void declareParameters()
  {
    rcl_interfaces::msg::ParameterDescriptor param_desc;
    MVCC_FLOATVALUE f_value;
    param_desc.integer_range.resize(1);
    param_desc.integer_range[0].step = 1;

    // Exposure time
    if (MV_CC_GetFloatValue(camera_handle_, "ExposureTime", &f_value) == MV_OK) {
      param_desc.description = "Exposure time in microseconds";
      param_desc.integer_range[0].from_value = f_value.fMin;
      param_desc.integer_range[0].to_value = f_value.fMax;
      if (!this->has_parameter("exposure_time")) {
        this->declare_parameter("exposure_time", 5000, param_desc);
      }
      double exposure = this->get_parameter("exposure_time").as_int();
      MV_CC_SetFloatValue(camera_handle_, "ExposureTime", exposure);
    } else {
      RCLCPP_WARN(this->get_logger(), "Cannot get ExposureTime range");
    }

    // Gain
    if (MV_CC_GetFloatValue(camera_handle_, "Gain", &f_value) == MV_OK) {
      param_desc.description = "Gain";
      param_desc.integer_range[0].from_value = f_value.fMin;
      param_desc.integer_range[0].to_value = f_value.fMax;
      if (!this->has_parameter("gain")) {
        this->declare_parameter("gain", f_value.fCurValue, param_desc);
      }
      double gain = this->get_parameter("gain").as_double();
      MV_CC_SetFloatValue(camera_handle_, "Gain", gain);
    } else {
      RCLCPP_WARN(this->get_logger(), "Cannot get Gain range");
    }
  }

  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    std::lock_guard<std::mutex> lock(camera_mutex_);
    if (!camera_handle_) {
      result.successful = false;
      result.reason = "Camera not connected";
      return result;
    }
    for (const auto & param : parameters) {
      if (param.get_name() == "exposure_time") {
        int status = MV_CC_SetFloatValue(camera_handle_, "ExposureTime", param.as_int());
        if (MV_OK != status) {
          result.successful = false;
          result.reason = "Failed to set exposure time, status = " + std::to_string(status);
        }
      } else if (param.get_name() == "gain") {
        int status = MV_CC_SetFloatValue(camera_handle_, "Gain", param.as_double());
        if (MV_OK != status) {
          result.successful = false;
          result.reason = "Failed to set gain, status = " + std::to_string(status);
        }
      } else {
        result.successful = false;
        result.reason = "Unknown parameter: " + param.get_name();
      }
    }
    return result;
  }

  // 成员变量
  sensor_msgs::msg::Image image_msg_;
  image_transport::CameraPublisher camera_pub_;
  void* camera_handle_ = nullptr;
  MV_IMAGE_BASIC_INFO img_info_;
  MV_CC_PIXEL_CONVERT_PARAM convert_param_;
  std::string camera_name_;
  std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
  sensor_msgs::msg::CameraInfo camera_info_msg_;

  int fail_count_ = 0;
  std::thread capture_thread_;
  OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;

  std::mutex camera_mutex_;
  std::mutex convert_mutex_;
};
}  // namespace hik_camera

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(hik_camera::HikCameraNode)

*/

//==================xiong：以上添加了相机断连机制==================



/*
#include "MvCameraControl.h"
// ROS
#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <unistd.h>
#include <cstdlib>
#include <atomic>      // 新增：用于原子标志
#include <thread>      // 新增：用于独立线程触发重连

namespace hik_camera
{
class HikCameraNode : public rclcpp::Node
{
public:
  explicit HikCameraNode(const rclcpp::NodeOptions & options) : Node("hik_camera", options)
  {
    RCLCPP_INFO(this->get_logger(), "Starting HikCameraNode!");

    // 连接相机（如果失败会无限重试）
    if (!connectCameraWithRetry()) {
      RCLCPP_FATAL(this->get_logger(), "Camera connection failed and no retry possible, exiting.");
      rclcpp::shutdown();
      return;
    }

    // 正常初始化其他部分...
    bool use_sensor_data_qos = this->declare_parameter("use_sensor_data_qos", true);
    auto qos = use_sensor_data_qos ? rmw_qos_profile_sensor_data : rmw_qos_profile_default;
    camera_pub_ = image_transport::create_camera_publisher(this, "image_raw", qos);

    camera_name_ = this->declare_parameter("camera_name", "narrow_stereo");
    camera_info_manager_ = std::make_unique<camera_info_manager::CameraInfoManager>(this, camera_name_);
    auto camera_info_url = this->declare_parameter("camera_info_url", "package://hik_camera/config/camera_info.yaml");
    if (camera_info_manager_->validateURL(camera_info_url)) {
      camera_info_manager_->loadCameraInfo(camera_info_url);
      camera_info_msg_ = camera_info_manager_->getCameraInfo();
    } else {
      RCLCPP_WARN(this->get_logger(), "Invalid camera info URL: %s", camera_info_url.c_str());
    }

    // 声明延迟检测参数（可调）
    max_allowed_frame_interval_ms_ = this->declare_parameter("max_allowed_frame_interval_ms", 200.0);
    last_frame_time_ = this->now();  // 初始化上一帧时间

    params_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&HikCameraNode::parametersCallback, this, std::placeholders::_1));

    capture_thread_ = std::thread{[this]() -> void {
      MV_FRAME_OUT out_frame;
      RCLCPP_INFO(this->get_logger(), "Publishing image!");

      image_msg_.header.frame_id = "camera_optical_frame";
      image_msg_.encoding = "rgb8";

      while (rclcpp::ok()) {
        int ret = MV_CC_GetImageBuffer(camera_handle_, &out_frame, 1000);
        if (MV_OK == ret) {
          // 成功获取图像
          // 修复：先根据图像尺寸分配缓冲区，再转换
          image_msg_.height = out_frame.stFrameInfo.nHeight;
          image_msg_.width = out_frame.stFrameInfo.nWidth;
          image_msg_.step = out_frame.stFrameInfo.nWidth * 3;
          image_msg_.data.resize(image_msg_.width * image_msg_.height * 3);

          convert_param_.pDstBuffer = image_msg_.data.data();
          convert_param_.nDstBufferSize = image_msg_.data.size();
          convert_param_.pSrcData = out_frame.pBufAddr;
          convert_param_.nSrcDataLen = out_frame.stFrameInfo.nFrameLen;
          convert_param_.enSrcPixelType = out_frame.stFrameInfo.enPixelType;

          MV_CC_ConvertPixelType(camera_handle_, &convert_param_);

          image_msg_.header.stamp = this->now();
          camera_info_msg_.header = image_msg_.header;
          camera_pub_.publish(image_msg_, camera_info_msg_);

          MV_CC_FreeImageBuffer(camera_handle_, &out_frame);
          fail_count_ = 0;  // 成功时重置失败计数

          // ---------- 新增：延迟检测 ----------
          rclcpp::Time now = this->now();
          double interval_ms = (now - last_frame_time_).seconds() * 1000.0;
          last_frame_time_ = now;

          if (interval_ms > max_allowed_frame_interval_ms_) {
            delay_detected_count_++;
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "Frame interval too large: %.1f ms (threshold %.1f ms), count=%d",
                interval_ms, max_allowed_frame_interval_ms_, delay_detected_count_);

            if (delay_detected_count_ >= 3 && !is_reconnecting_.load()) {
              RCLCPP_FATAL(this->get_logger(), "Severe delay detected, triggering camera soft reset...");
              // 异步触发重连，避免阻塞取图线程
              std::thread([this]() {
                this->reconnectCamera();
              }).detach();
              delay_detected_count_ = 0;  // 重置计数，避免重复触发
            }
          } else {
            // 帧间隔正常，重置延迟计数
            delay_detected_count_ = 0;
          }
          // ----------------------------------
        } else {
          RCLCPP_WARN(this->get_logger(), "Get buffer failed! nRet: [%x]", ret);
          fail_count_++;
          if (fail_count_ >= 5) {
            RCLCPP_FATAL(this->get_logger(), "Camera streaming failed repeatedly, attempting to reconnect...");
            // 尝试重连（内部已防重入）
            if (reconnectCamera()) {
              RCLCPP_INFO(this->get_logger(), "Camera reconnected successfully.");
              fail_count_ = 0;
            } else {
              RCLCPP_ERROR(this->get_logger(), "Camera reconnection failed, will retry again later.");
              // 继续循环，下一次循环会再次尝试重连
            }
          }
        }
      }
    }};
  }

  ~HikCameraNode() override
  {
    if (capture_thread_.joinable()) {
      capture_thread_.join();
    }
    disconnectCamera();
    RCLCPP_INFO(this->get_logger(), "HikCameraNode destroyed!");
  }

private:
  // 连接相机，如果失败会一直重试（无限等待）
  bool connectCameraWithRetry()
  {
    while (rclcpp::ok()) {
      if (connectCamera()) {
        return true;
      }
      RCLCPP_ERROR(this->get_logger(), "Camera connection failed, retrying in 100 ms");
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return false;
  }

  // 连接相机（单次尝试，不循环）
  bool connectCamera()
  {
    std::lock_guard<std::mutex> lock(camera_mutex_);

    MV_CC_DEVICE_INFO_LIST device_list;
    int ret = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
    if (ret != MV_OK) {
      RCLCPP_ERROR(this->get_logger(), "Enumeration failed, error: [%x]", ret);
      return false;
    }

    if (device_list.nDeviceNum == 0) {
      RCLCPP_ERROR(this->get_logger(), "No camera found");
      return false;
    }

    ret = MV_CC_CreateHandle(&camera_handle_, device_list.pDeviceInfo[0]);
    if (ret != MV_OK) {
      RCLCPP_ERROR(this->get_logger(), "Create handle failed, error: [%x]", ret);
      return false;
    }

    ret = MV_CC_OpenDevice(camera_handle_);
    if (ret != MV_OK) {
      RCLCPP_ERROR(this->get_logger(), "Open device failed, error: [%x]", ret);
      MV_CC_DestroyHandle(&camera_handle_);
      camera_handle_ = nullptr;
      return false;
    }

    ret = MV_CC_GetImageInfo(camera_handle_, &img_info_);
    if (ret != MV_OK) {
      RCLCPP_ERROR(this->get_logger(), "Get image info failed, error: [%x]", ret);
      MV_CC_CloseDevice(camera_handle_);
      MV_CC_DestroyHandle(&camera_handle_);
      camera_handle_ = nullptr;
      return false;
    }

    // 初始化转换参数
    convert_param_.nWidth = img_info_.nWidthValue;
    convert_param_.nHeight = img_info_.nHeightValue;
    convert_param_.enDstPixelType = PixelType_Gvsp_RGB8_Packed;

    // 声明并设置相机参数
    declareParameters();

    ret = MV_CC_StartGrabbing(camera_handle_);
    if (ret != MV_OK) {
      RCLCPP_ERROR(this->get_logger(), "Start grabbing failed, error: [%x]", ret);
      MV_CC_CloseDevice(camera_handle_);
      MV_CC_DestroyHandle(&camera_handle_);
      camera_handle_ = nullptr;
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "Camera connected successfully");
    return true;
  }

  // 重连相机：断开现有连接，然后尝试重新连接（会循环重试直到成功）
  // 增加防重入保护
  bool reconnectCamera()
  {
    // 防止多个线程同时重连
    bool expected = false;
    if (!is_reconnecting_.compare_exchange_strong(expected, true)) {
      RCLCPP_WARN(this->get_logger(), "Reconnection already in progress, skipping.");
      return false;
    }

    // 确保退出时重置标志
    auto reset_flag = [this]() { is_reconnecting_ = false; };
    std::shared_ptr<void> guard(nullptr, [&](void*) { reset_flag(); });

    RCLCPP_INFO(this->get_logger(), "Attempting to reconnect camera...");
    disconnectCamera();  // 释放当前资源
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    bool success = connectCameraWithRetry();  // 一直重试直到成功
    if (success) {
      // 重连成功后重置延迟相关状态
      last_frame_time_ = this->now();
      delay_detected_count_ = 0;
      fail_count_ = 0;
    }
    return success;
  }

  void disconnectCamera()
  {
    std::lock_guard<std::mutex> lock(camera_mutex_);
    if (camera_handle_) {
      MV_CC_StopGrabbing(camera_handle_);
      MV_CC_CloseDevice(camera_handle_);
      MV_CC_DestroyHandle(&camera_handle_);
      camera_handle_ = nullptr;
    }
  }

  void declareParameters()
  {
    rcl_interfaces::msg::ParameterDescriptor param_desc;
    MVCC_FLOATVALUE f_value;
    param_desc.integer_range.resize(1);
    param_desc.integer_range[0].step = 1;

    // Exposure time
    if (MV_CC_GetFloatValue(camera_handle_, "ExposureTime", &f_value) == MV_OK) {
      param_desc.description = "Exposure time in microseconds";
      param_desc.integer_range[0].from_value = f_value.fMin;
      param_desc.integer_range[0].to_value = f_value.fMax;
      if (!this->has_parameter("exposure_time")) {
        this->declare_parameter("exposure_time", 5000, param_desc);
      }
      double exposure = this->get_parameter("exposure_time").as_int();
      MV_CC_SetFloatValue(camera_handle_, "ExposureTime", exposure);
    } else {
      RCLCPP_WARN(this->get_logger(), "Cannot get ExposureTime range");
    }

    // Gain
    if (MV_CC_GetFloatValue(camera_handle_, "Gain", &f_value) == MV_OK) {
      param_desc.description = "Gain";
      param_desc.integer_range[0].from_value = f_value.fMin;
      param_desc.integer_range[0].to_value = f_value.fMax;
      if (!this->has_parameter("gain")) {
        this->declare_parameter("gain", f_value.fCurValue, param_desc);
      }
      double gain = this->get_parameter("gain").as_double();
      MV_CC_SetFloatValue(camera_handle_, "Gain", gain);
    } else {
      RCLCPP_WARN(this->get_logger(), "Cannot get Gain range");
    }

    // 可选：将延迟阈值设为可配置参数
    if (!this->has_parameter("max_allowed_frame_interval_ms")) {
      this->declare_parameter("max_allowed_frame_interval_ms", 200.0);
    }
  }

  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    std::lock_guard<std::mutex> lock(camera_mutex_);
    if (!camera_handle_) {
      result.successful = false;
      result.reason = "Camera not connected";
      return result;
    }
    for (const auto & param : parameters) {
      if (param.get_name() == "exposure_time") {
        int status = MV_CC_SetFloatValue(camera_handle_, "ExposureTime", param.as_int());
        if (MV_OK != status) {
          result.successful = false;
          result.reason = "Failed to set exposure time, status = " + std::to_string(status);
        }
      } else if (param.get_name() == "gain") {
        int status = MV_CC_SetFloatValue(camera_handle_, "Gain", param.as_double());
        if (MV_OK != status) {
          result.successful = false;
          result.reason = "Failed to set gain, status = " + std::to_string(status);
        }
      } else if (param.get_name() == "max_allowed_frame_interval_ms") {
        max_allowed_frame_interval_ms_ = param.as_double();
      } else {
        result.successful = false;
        result.reason = "Unknown parameter: " + param.get_name();
      }
    }
    return result;
  }

  // 成员变量
  sensor_msgs::msg::Image image_msg_;
  image_transport::CameraPublisher camera_pub_;
  void* camera_handle_ = nullptr;
  MV_IMAGE_BASIC_INFO img_info_;
  MV_CC_PIXEL_CONVERT_PARAM convert_param_;
  std::string camera_name_;
  std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
  sensor_msgs::msg::CameraInfo camera_info_msg_;

  int fail_count_ = 0;
  std::thread capture_thread_;
  OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;

  std::mutex camera_mutex_;
  std::mutex convert_mutex_;

  // 新增：延迟检测相关
  rclcpp::Time last_frame_time_;
  int delay_detected_count_ = 0;
  double max_allowed_frame_interval_ms_ = 200.0;
  std::atomic<bool> is_reconnecting_{false};
};
}  // namespace hik_camera

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(hik_camera::HikCameraNode)

//==========xiong：以上为包含 相机断连/相机延迟 机制===========（大概有效）



#include "MvCameraControl.h"
// ROS
#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <unistd.h>
#include <cstdlib>
#include <chrono>
#include <thread>
#include <mutex>

namespace hik_camera
{

class HikCameraNode : public rclcpp::Node
{
public:
  explicit HikCameraNode(const rclcpp::NodeOptions & options) : Node("hik_camera", options)
  {
    RCLCPP_INFO(this->get_logger(), "Starting HikCameraNode!");

    // 连接相机（如果失败会无限重试，符合需求）
    if (!connectCameraWithRetry()) {
      RCLCPP_FATAL(this->get_logger(), "Camera connection failed and no retry possible, exiting.");
      rclcpp::shutdown();
      return;
    }

    // 正常初始化ROS发布器等
    bool use_sensor_data_qos = this->declare_parameter("use_sensor_data_qos", true);
    auto qos = use_sensor_data_qos ? rmw_qos_profile_sensor_data : rmw_qos_profile_default;
    camera_pub_ = image_transport::create_camera_publisher(this, "image_raw", qos);

    camera_name_ = this->declare_parameter("camera_name", "narrow_stereo");
    camera_info_manager_ = std::make_unique<camera_info_manager::CameraInfoManager>(this, camera_name_);
    auto camera_info_url = this->declare_parameter("camera_info_url", "package://hik_camera/config/camera_info.yaml");
    if (camera_info_manager_->validateURL(camera_info_url)) {
      camera_info_manager_->loadCameraInfo(camera_info_url);
      camera_info_msg_ = camera_info_manager_->getCameraInfo();
    } else {
      RCLCPP_WARN(this->get_logger(), "Invalid camera info URL: %s", camera_info_url.c_str());
    }

    params_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&HikCameraNode::parametersCallback, this, std::placeholders::_1));

    // 启动采集线程
    capture_thread_ = std::thread{[this]() -> void {
      MV_FRAME_OUT out_frame;
      RCLCPP_INFO(this->get_logger(), "Publishing image!");

      // 图像消息固定字段
      image_msg_.header.frame_id = "camera_optical_frame";
      image_msg_.encoding = "rgb8";

      // 记录上一次的图像尺寸，用于检测变化
      int last_width = 0, last_height = 0;

      while (rclcpp::ok()) {
        int ret = MV_OK;
        // 加锁获取图像
        {
          std::lock_guard<std::mutex> lock(camera_mutex_);
          if (!camera_handle_) {
            // 相机未连接，稍等重试
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
          }
          ret = MV_CC_GetImageBuffer(camera_handle_, &out_frame, 1000);
        }

        if (MV_OK == ret) {
          // 成功获取图像
          int width = out_frame.stFrameInfo.nWidth;
          int height = out_frame.stFrameInfo.nHeight;
          // 检测尺寸变化，动态更新转换参数和图像缓冲区
          if (width != last_width || height != last_height) {
            std::lock_guard<std::mutex> lock(camera_mutex_);
            convert_param_.nWidth = width;
            convert_param_.nHeight = height;
            // 预分配图像数据空间（RGB8 需要 width*height*3 字节）
            image_msg_.width = width;
            image_msg_.height = height;
            image_msg_.step = width * 3;
            image_msg_.data.resize(width * height * 3);
            RCLCPP_INFO(this->get_logger(), "Image size changed to %dx%d", width, height);
            last_width = width;
            last_height = height;
          }

          // 先确保缓冲区大小正确
          image_msg_.data.resize(width * height * 3);
          // 设置转换参数
          convert_param_.pDstBuffer = image_msg_.data.data();
          convert_param_.nDstBufferSize = image_msg_.data.size();
          convert_param_.pSrcData = out_frame.pBufAddr;
          convert_param_.nSrcDataLen = out_frame.stFrameInfo.nFrameLen;
          convert_param_.enSrcPixelType = out_frame.stFrameInfo.enPixelType;
          // 执行像素格式转换（需要加锁，因为内部可能访问相机句柄）
          {
            std::lock_guard<std::mutex> lock(camera_mutex_);
            ret = MV_CC_ConvertPixelType(camera_handle_, &convert_param_);
          }
          if (ret != MV_OK) {
            RCLCPP_WARN(this->get_logger(), "Convert pixel type failed: [%x]", ret);
            // 释放图像缓冲区后继续
            std::lock_guard<std::mutex> lock(camera_mutex_);
            if (camera_handle_) {
              MV_CC_FreeImageBuffer(camera_handle_, &out_frame);
            }
            fail_count_++;
            if (fail_count_ >= 5) {
              triggerReconnect();
            }
            continue;
          }

          // 填充ROS消息
          image_msg_.header.stamp = this->now();  // 暂时使用系统时间，后面会用硬件时间戳覆盖
          // 获取硬件时间戳（微秒）
          uint64_t device_timestamp_us = 
            (static_cast<uint64_t>(out_frame.stFrameInfo.nDevTimeStampHigh) << 32) | 
            out_frame.stFrameInfo.nDevTimeStampLow;
          // 如果硬件时间戳有效（非0），则使用它；否则保留系统时间
          if (device_timestamp_us != 0) {
            // 注意：海康时间戳通常是微秒，转换为ROS时间（秒+纳秒）
            uint64_t sec = device_timestamp_us / 1000000ULL;
            uint64_t nsec = (device_timestamp_us % 1000000ULL) * 1000ULL;
            image_msg_.header.stamp = rclcpp::Time(sec, nsec);
          } else {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                                 "Device timestamp is zero, using system time.");
          }

          // 发布
          camera_info_msg_.header = image_msg_.header;
          camera_pub_.publish(image_msg_, camera_info_msg_);

          // 释放图像缓冲区
          {
            std::lock_guard<std::mutex> lock(camera_mutex_);
            if (camera_handle_) {
              MV_CC_FreeImageBuffer(camera_handle_, &out_frame);
            }
          }
          fail_count_ = 0;  // 成功时重置失败计数
        } else {
          RCLCPP_WARN(this->get_logger(), "Get buffer failed! nRet: [%x]", ret);
          fail_count_++;
          if (fail_count_ >= 5) {
            triggerReconnect();
          }
        }
      }
    }};
  }

  ~HikCameraNode() override
  {
    if (capture_thread_.joinable()) {
      capture_thread_.join();
    }
    disconnectCamera();
    RCLCPP_INFO(this->get_logger(), "HikCameraNode destroyed!");
  }

private:
  // 连接相机，如果失败会一直重试（无限等待）
  bool connectCameraWithRetry()
  {
    while (rclcpp::ok()) {
      if (connectCamera()) {
        return true;
      }
      RCLCPP_ERROR(this->get_logger(), "Camera connection failed, retrying in 100 milliseconds");
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return false;
  }

  // 连接相机（单次尝试，不循环）
  bool connectCamera()
  {
    std::lock_guard<std::mutex> lock(camera_mutex_);

    MV_CC_DEVICE_INFO_LIST device_list;
    int ret = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
    if (ret != MV_OK) {
      RCLCPP_ERROR(this->get_logger(), "Enumeration failed, error: [%x]", ret);
      return false;
    }

    if (device_list.nDeviceNum == 0) {
      RCLCPP_ERROR(this->get_logger(), "No camera found");
      return false;
    }

    ret = MV_CC_CreateHandle(&camera_handle_, device_list.pDeviceInfo[0]);
    if (ret != MV_OK) {
      RCLCPP_ERROR(this->get_logger(), "Create handle failed, error: [%x]", ret);
      return false;
    }

    ret = MV_CC_OpenDevice(camera_handle_);
    if (ret != MV_OK) {
      RCLCPP_ERROR(this->get_logger(), "Open device failed, error: [%x]", ret);
      MV_CC_DestroyHandle(&camera_handle_);
      camera_handle_ = nullptr;
      return false;
    }

    ret = MV_CC_GetImageInfo(camera_handle_, &img_info_);
    if (ret != MV_OK) {
      RCLCPP_ERROR(this->get_logger(), "Get image info failed, error: [%x]", ret);
      MV_CC_CloseDevice(camera_handle_);
      MV_CC_DestroyHandle(&camera_handle_);
      camera_handle_ = nullptr;
      return false;
    }

    // 初始化转换参数（仅设置目标像素类型，宽高会在采集时动态更新）
    convert_param_.enDstPixelType = PixelType_Gvsp_RGB8_Packed;
    // 预分配图像数据空间（使用最大分辨率）
    image_msg_.data.reserve(img_info_.nHeightMax * img_info_.nWidthMax * 3);

    // 声明并设置相机参数
    declareParameters();

    ret = MV_CC_StartGrabbing(camera_handle_);
    if (ret != MV_OK) {
      RCLCPP_ERROR(this->get_logger(), "Start grabbing failed, error: [%x]", ret);
      MV_CC_CloseDevice(camera_handle_);
      MV_CC_DestroyHandle(&camera_handle_);
      camera_handle_ = nullptr;
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "Camera connected successfully");
    return true;
  }

  // 触发重连（在失败计数达到阈值时调用）
  void triggerReconnect()
  {
    RCLCPP_INFO(this->get_logger(), "Triggering camera reconnect...");
    // 注意：不要在持有锁的情况下调用 reconnectCamera，因为 reconnectCamera 会尝试获取锁
    // 这里我们启动一个异步任务来执行重连，避免死锁
    std::thread([this]() {
      // 稍微延迟，避免过于频繁
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      if (reconnectCamera()) {
        RCLCPP_INFO(this->get_logger(), "Camera reconnected successfully.");
        // 重连成功后，重置失败计数（已在 reconnectCamera 内部重置）
      } else {
        RCLCPP_ERROR(this->get_logger(), "Camera reconnection failed, will retry later.");
      }
    }).detach();
  }

  // 重连相机：断开现有连接，然后尝试重新连接（会循环重试直到成功）
  bool reconnectCamera()
  {
    RCLCPP_INFO(this->get_logger(), "Attempting to reconnect camera...");
    disconnectCamera();  // 释放当前资源
    // 等待一段时间让设备稳定
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    bool success = connectCameraWithRetry();  // 一直重试直到成功
    if (success) {
      fail_count_ = 0;
    }
    return success;
  }

  void disconnectCamera()
  {
    std::lock_guard<std::mutex> lock(camera_mutex_);
    if (camera_handle_) {
      MV_CC_StopGrabbing(camera_handle_);
      MV_CC_CloseDevice(camera_handle_);
      MV_CC_DestroyHandle(&camera_handle_);
      camera_handle_ = nullptr;
    }
  }

  void declareParameters()
  {
    rcl_interfaces::msg::ParameterDescriptor param_desc;
    MVCC_FLOATVALUE f_value;
    param_desc.integer_range.resize(1);
    param_desc.integer_range[0].step = 1;

    // Exposure time
    if (MV_CC_GetFloatValue(camera_handle_, "ExposureTime", &f_value) == MV_OK) {
      param_desc.description = "Exposure time in microseconds";
      param_desc.integer_range[0].from_value = f_value.fMin;
      param_desc.integer_range[0].to_value = f_value.fMax;
      if (!this->has_parameter("exposure_time")) {
        this->declare_parameter("exposure_time", 5000, param_desc);
      }
      double exposure = this->get_parameter("exposure_time").as_int();
      MV_CC_SetFloatValue(camera_handle_, "ExposureTime", exposure);
    } else {
      RCLCPP_WARN(this->get_logger(), "Cannot get ExposureTime range");
    }

    // Gain
    if (MV_CC_GetFloatValue(camera_handle_, "Gain", &f_value) == MV_OK) {
      param_desc.description = "Gain";
      param_desc.integer_range[0].from_value = f_value.fMin;
      param_desc.integer_range[0].to_value = f_value.fMax;
      if (!this->has_parameter("gain")) {
        this->declare_parameter("gain", f_value.fCurValue, param_desc);
      }
      double gain = this->get_parameter("gain").as_double();
      MV_CC_SetFloatValue(camera_handle_, "Gain", gain);
    } else {
      RCLCPP_WARN(this->get_logger(), "Cannot get Gain range");
    }
  }

  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    std::lock_guard<std::mutex> lock(camera_mutex_);
    if (!camera_handle_) {
      result.successful = false;
      result.reason = "Camera not connected";
      return result;
    }
    for (const auto & param : parameters) {
      if (param.get_name() == "exposure_time") {
        int status = MV_CC_SetFloatValue(camera_handle_, "ExposureTime", param.as_int());
        if (MV_OK != status) {
          result.successful = false;
          result.reason = "Failed to set exposure time, status = " + std::to_string(status);
        }
      } else if (param.get_name() == "gain") {
        int status = MV_CC_SetFloatValue(camera_handle_, "Gain", param.as_double());
        if (MV_OK != status) {
          result.successful = false;
          result.reason = "Failed to set gain, status = " + std::to_string(status);
        }
      } else {
        result.successful = false;
        result.reason = "Unknown parameter: " + param.get_name();
      }
    }
    return result;
  }

  // 成员变量
  sensor_msgs::msg::Image image_msg_;
  image_transport::CameraPublisher camera_pub_;
  void* camera_handle_ = nullptr;
  MV_IMAGE_BASIC_INFO img_info_;
  MV_CC_PIXEL_CONVERT_PARAM convert_param_;
  std::string camera_name_;
  std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
  sensor_msgs::msg::CameraInfo camera_info_msg_;

  int fail_count_ = 0;
  std::thread capture_thread_;
  OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;

  std::mutex camera_mutex_;  // 保护 camera_handle_ 以及所有 SDK 调用
};
}  // namespace hik_camera

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(hik_camera::HikCameraNode)


####//====================//####
