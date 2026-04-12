#include "MvCameraControl.h"
#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <unistd.h>
#include <mutex>
#include <vector>
#include <thread>
#include <string>
#include <chrono>
#include <memory>   // for std::unique_ptr

namespace hik_camera
{

// 辅助函数：将 unsigned char 数组转换为 std::string
static std::string ucharToString(const unsigned char* arr, size_t len)
{
    // 找到第一个 '\0' 位置
    size_t size = 0;
    while (size < len && arr[size] != '\0') ++size;
    return std::string(reinterpret_cast<const char*>(arr), size);
}

// 获取设备序列号（USB或GigE）
static std::string getDeviceSerialNumber(MV_CC_DEVICE_INFO* pDeviceInfo)
{
    if (pDeviceInfo->nTLayerType == MV_USB_DEVICE) {
        return ucharToString(pDeviceInfo->SpecialInfo.stUsb3VInfo.chSerialNumber,
                             sizeof(pDeviceInfo->SpecialInfo.stUsb3VInfo.chSerialNumber));
    } else if (pDeviceInfo->nTLayerType == MV_GIGE_DEVICE) {
        return ucharToString(pDeviceInfo->SpecialInfo.stGigEInfo.chSerialNumber,
                             sizeof(pDeviceInfo->SpecialInfo.stGigEInfo.chSerialNumber));
    }
    return "";
}

class HikDualCameraNode : public rclcpp::Node
{
public:
  explicit HikDualCameraNode(const rclcpp::NodeOptions & options)
    : Node("hik_dual_camera", options)
  {
    RCLCPP_INFO(this->get_logger(), "启动双相机驱动 HikDualCameraNode");

    // 1. 声明ROS参数（序列号、内参文件、帧率等）
    declareParameters();

    // 2. 连接相机（失败则无限重试）
    if (!connectAllCamerasWithRetry()) {
      RCLCPP_FATAL(this->get_logger(), "无法连接任何相机，节点退出");
      rclcpp::shutdown();
      return;
    }

    // 3. 启动采集线程
    startCaptureThreads();

    RCLCPP_INFO(this->get_logger(), "双相机初始化完成，已连接 %zu 个相机", camera_handles_.size());
  }

  ~HikDualCameraNode() override
  {
    for (auto &t : capture_threads_) {
      if (t.joinable()) t.join();
    }
    disconnectAllCameras();
    RCLCPP_INFO(this->get_logger(), "双相机驱动已关闭");
  }

private:
  // ======================== 参数声明 ========================
  void declareParameters()
  {
    this->declare_parameter("camera0_serial", "");
    this->declare_parameter("camera0_info_url", "");
    this->declare_parameter("camera0_exposure", 5000);
    this->declare_parameter("camera0_gain", 0.0);
    this->declare_parameter("camera0_fps", 30.0);   // 新增：帧率限制

    this->declare_parameter("camera1_serial", "");
    this->declare_parameter("camera1_info_url", "");
    this->declare_parameter("camera1_exposure", 5000);
    this->declare_parameter("camera1_gain", 0.0);
    this->declare_parameter("camera1_fps", 30.0);   // 新增：帧率限制
  }

  // ======================== 多相机连接（无限重试） ========================
  bool connectAllCamerasWithRetry()
  {
    while (rclcpp::ok()) {
      if (connectAllCamerasOnce()) {
        return true;
      }
      RCLCPP_ERROR(this->get_logger(), "相机连接失败，2秒后重试...");
      std::this_thread::sleep_for(std::chrono::seconds(2));
    }
    return false;
  }

  bool connectAllCamerasOnce()
  {
    // 枚举所有设备
    MV_CC_DEVICE_INFO_LIST device_list;
    int ret = MV_CC_EnumDevices(MV_USB_DEVICE | MV_GIGE_DEVICE, &device_list);
    if (ret != MV_OK || device_list.nDeviceNum == 0) {
      RCLCPP_ERROR(this->get_logger(), "未找到任何相机 (枚举失败: %x)", ret);
      return false;
    }

    // 获取用户指定的序列号（若提供）
    std::string sn0 = this->get_parameter("camera0_serial").as_string();
    std::string sn1 = this->get_parameter("camera1_serial").as_string();

    // 用于存储已匹配的索引，避免重复使用同一设备
    std::vector<int> matched_indices;

    auto tryConnect = [&](int cam_idx, const std::string& target_sn) -> bool {
        for (unsigned int i = 0; i < device_list.nDeviceNum; ++i) {
            if (std::find(matched_indices.begin(), matched_indices.end(), i) != matched_indices.end())
                continue;

            MV_CC_DEVICE_INFO* dev = device_list.pDeviceInfo[i];
            std::string dev_sn = getDeviceSerialNumber(dev);
            if (!target_sn.empty() && dev_sn != target_sn)
                continue;

            void* handle = nullptr;
            ret = MV_CC_CreateHandle(&handle, dev);
            if (ret != MV_OK) {
                RCLCPP_WARN(this->get_logger(), "相机%d 创建句柄失败: %x", cam_idx, ret);
                continue;
            }
            ret = MV_CC_OpenDevice(handle);
            if (ret != MV_OK) {
                RCLCPP_WARN(this->get_logger(), "相机%d 打开设备失败: %x", cam_idx, ret);
                MV_CC_DestroyHandle(&handle);
                continue;
            }

            MV_IMAGE_BASIC_INFO img_info;
            ret = MV_CC_GetImageInfo(handle, &img_info);
            if (ret != MV_OK) {
                RCLCPP_WARN(this->get_logger(), "相机%d 获取图像信息失败: %x", cam_idx, ret);
                MV_CC_CloseDevice(handle);
                MV_CC_DestroyHandle(&handle);
                continue;
            }

            ret = MV_CC_SetEnumValue(handle, "TriggerMode", 0);
            if (ret != MV_OK) {
                RCLCPP_WARN(this->get_logger(), "相机%d 设置触发模式失败: %x", cam_idx, ret);
            }

            double exposure = this->get_parameter("camera" + std::to_string(cam_idx) + "_exposure").as_int();
            double gain = this->get_parameter("camera" + std::to_string(cam_idx) + "_gain").as_double();
            MV_CC_SetFloatValue(handle, "ExposureTime", exposure);
            MV_CC_SetFloatValue(handle, "Gain", gain);

            ret = MV_CC_StartGrabbing(handle);
            if (ret != MV_OK) {
                RCLCPP_WARN(this->get_logger(), "相机%d 启动取流失败: %x", cam_idx, ret);
                MV_CC_CloseDevice(handle);
                MV_CC_DestroyHandle(&handle);
                continue;
            }

            // 保存相机信息
            camera_handles_.push_back(handle);
            img_infos_.push_back(img_info);
            fail_counts_.push_back(0);
            serial_numbers_.push_back(dev_sn);
            // 存储帧率限制
            double fps = this->get_parameter("camera" + std::to_string(cam_idx) + "_fps").as_double();
            fps_limits_.push_back(fps > 0.0 ? fps : 30.0);   // 默认30，若<=0则不限制

            camera_mutexes_.push_back(std::make_unique<std::mutex>());

            std::string topic = "camera_" + std::to_string(cam_idx) + "/image_raw";
            rclcpp::QoS qos(rclcpp::KeepLast(5));
            qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
            qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
            auto pub = image_transport::create_camera_publisher(
                this, topic, qos.get_rmw_qos_profile());
            camera_pubs_.push_back(pub);

            std::string info_url = this->get_parameter("camera" + std::to_string(cam_idx) + "_info_url").as_string();
            auto info_mgr = std::make_unique<camera_info_manager::CameraInfoManager>(this, "camera_" + std::to_string(cam_idx));
            sensor_msgs::msg::CameraInfo cam_info_msg;
            if (!info_url.empty() && info_mgr->validateURL(info_url)) {
                info_mgr->loadCameraInfo(info_url);
                cam_info_msg = info_mgr->getCameraInfo();
                RCLCPP_INFO(this->get_logger(), "相机%d 已加载标定文件: %s", cam_idx, info_url.c_str());
            } else {
                RCLCPP_WARN(this->get_logger(), "相机%d 未提供有效标定文件，将发布空内参", cam_idx);
            }
            info_managers_.push_back(std::move(info_mgr));
            camera_info_msgs_.push_back(cam_info_msg);

            MV_CC_PIXEL_CONVERT_PARAM param{};
            param.enDstPixelType = PixelType_Gvsp_RGB8_Packed;
            convert_params_.push_back(param);

            sensor_msgs::msg::Image img_msg;
            img_msg.header.frame_id = "camera_" + std::to_string(cam_idx) + "_optical_frame";
            img_msg.encoding = "rgb8";
            image_msgs_.push_back(img_msg);

            matched_indices.push_back(i);
            RCLCPP_INFO(this->get_logger(), "相机%d 连接成功 (序列号: %s, fps限制: %.1f)", cam_idx, dev_sn.c_str(), fps_limits_.back());
            return true;
        }
        return false;
    };

    bool ok0 = tryConnect(0, sn0);
    bool ok1 = tryConnect(1, sn1);

    if (!ok0 && !ok1) {
        disconnectAllCameras();
        return false;
    }

    if ((!sn0.empty() && !ok0) || (!sn1.empty() && !ok1)) {
        RCLCPP_WARN(this->get_logger(), "部分相机未能按指定序列号连接");
    }

    return true;
  }

  void disconnectAllCameras()
  {
    for (size_t i = 0; i < camera_handles_.size(); ++i) {
      std::lock_guard<std::mutex> lock(*(camera_mutexes_[i]));
      if (camera_handles_[i]) {
        MV_CC_StopGrabbing(camera_handles_[i]);
        MV_CC_CloseDevice(camera_handles_[i]);
        MV_CC_DestroyHandle(&camera_handles_[i]);
        camera_handles_[i] = nullptr;
      }
    }
    camera_handles_.clear();
    camera_mutexes_.clear();
    img_infos_.clear();
    convert_params_.clear();
    image_msgs_.clear();
    camera_info_msgs_.clear();
    camera_pubs_.clear();
    info_managers_.clear();
    fail_counts_.clear();
    serial_numbers_.clear();
    fps_limits_.clear();
  }

  // ======================== 采集线程 ========================
  void startCaptureThreads()
  {
    for (size_t i = 0; i < camera_handles_.size(); ++i) {
      capture_threads_.emplace_back(&HikDualCameraNode::captureThread, this, i);
    }
  }

  void captureThread(size_t cam_idx)
  {
    auto& handle = camera_handles_[cam_idx];
    auto& image_msg = image_msgs_[cam_idx];
    auto& camera_pub = camera_pubs_[cam_idx];
    auto& camera_info_msg = camera_info_msgs_[cam_idx];
    auto& convert_param = convert_params_[cam_idx];
    auto& fail_count = fail_counts_[cam_idx];
    auto& mutex = camera_mutexes_[cam_idx];
    double fps_limit = fps_limits_[cam_idx];

    int last_w = 0, last_h = 0;
    // 帧率控制相关变量
    std::chrono::steady_clock::time_point last_pub_time = std::chrono::steady_clock::now();
    const std::chrono::duration<double> frame_interval(1.0 / fps_limit);

    while (rclcpp::ok()) {
      MV_FRAME_OUT out_frame{};
      int ret;

      {
        std::lock_guard<std::mutex> lock(*mutex);
        if (!handle) {
          std::this_thread::sleep_for(std::chrono::milliseconds(500));
          continue;
        }
        ret = MV_CC_GetImageBuffer(handle, &out_frame, 200);
      }

      if (ret == MV_OK) {
        int w = out_frame.stFrameInfo.nWidth;
        int h = out_frame.stFrameInfo.nHeight;

        if (w != last_w || h != last_h) {
          image_msg.width = w;
          image_msg.height = h;
          image_msg.step = w * 3;
          image_msg.data.resize(w * h * 3);
          last_w = w; last_h = h;
        }

        convert_param.nWidth = w;
        convert_param.nHeight = h;
        convert_param.pSrcData = out_frame.pBufAddr;
        convert_param.nSrcDataLen = out_frame.stFrameInfo.nFrameLen;
        convert_param.enSrcPixelType = out_frame.stFrameInfo.enPixelType;
        convert_param.pDstBuffer = image_msg.data.data();
        convert_param.nDstBufferSize = image_msg.data.size();

        {
          std::lock_guard<std::mutex> lock(*mutex);
          ret = MV_CC_ConvertPixelType(handle, &convert_param);
        }
        if (ret != MV_OK) {
          RCLCPP_WARN(this->get_logger(), "相机%zu 像素转换失败: %x", cam_idx, ret);
          std::lock_guard<std::mutex> lock(*mutex);
          if (handle) MV_CC_FreeImageBuffer(handle, &out_frame);
          fail_count++;
          if (fail_count > 5) {
            RCLCPP_ERROR(this->get_logger(), "相机%zu 连续失败，触发重连", cam_idx);
            reconnectCamera(cam_idx);
            fail_count = 0;
          }
          continue;
        }

        // 硬件时间戳回退
        // 统一使用系统时钟，避免硬件时间戳与系统时钟不同步导致 TF 查询失败
image_msg.header.stamp = this->now();

        // 帧率控制：计算距离上一次发布的时间，若未到间隔则睡眠
        if (fps_limit > 0.0) {
            auto now = std::chrono::steady_clock::now();
            auto elapsed = now - last_pub_time;
            if (elapsed < frame_interval) {
                auto sleep_duration = frame_interval - elapsed;
                std::this_thread::sleep_for(sleep_duration);
            }
            last_pub_time = std::chrono::steady_clock::now();
        }

        // 发布
        camera_info_msg.header = image_msg.header;
        camera_pub.publish(image_msg, camera_info_msg);

        {
          std::lock_guard<std::mutex> lock(*mutex);
          if (handle) MV_CC_FreeImageBuffer(handle, &out_frame);
        }
        fail_count = 0;
      } else {
        fail_count++;
        if (fail_count > 10) {
          RCLCPP_ERROR(this->get_logger(), "相机%zu 取流超时，尝试重连", cam_idx);
          reconnectCamera(cam_idx);
          fail_count = 0;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    }
  }

  // ======================== 重连机制（基于序列号） ========================
  void reconnectCamera(size_t cam_idx)
  {
    RCLCPP_INFO(this->get_logger(), "相机%zu 开始重连...", cam_idx);
    {
      std::lock_guard<std::mutex> lock(*(camera_mutexes_[cam_idx]));
      if (camera_handles_[cam_idx]) {
        MV_CC_StopGrabbing(camera_handles_[cam_idx]);
        MV_CC_CloseDevice(camera_handles_[cam_idx]);
        MV_CC_DestroyHandle(&camera_handles_[cam_idx]);
        camera_handles_[cam_idx] = nullptr;
      }
    }

    std::string target_sn = serial_numbers_[cam_idx];
    while (rclcpp::ok()) {
      MV_CC_DEVICE_INFO_LIST device_list;
      if (MV_CC_EnumDevices(MV_USB_DEVICE | MV_GIGE_DEVICE, &device_list) != MV_OK) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        continue;
      }
      bool found = false;
      for (unsigned int i = 0; i < device_list.nDeviceNum; ++i) {
        std::string dev_sn = getDeviceSerialNumber(device_list.pDeviceInfo[i]);
        if (dev_sn == target_sn) {
          void* new_handle = nullptr;
          if (MV_CC_CreateHandle(&new_handle, device_list.pDeviceInfo[i]) == MV_OK &&
              MV_CC_OpenDevice(new_handle) == MV_OK) {
            double exposure = this->get_parameter("camera" + std::to_string(cam_idx) + "_exposure").as_int();
            double gain = this->get_parameter("camera" + std::to_string(cam_idx) + "_gain").as_double();
            MV_CC_SetFloatValue(new_handle, "ExposureTime", exposure);
            MV_CC_SetFloatValue(new_handle, "Gain", gain);
            MV_CC_SetEnumValue(new_handle, "TriggerMode", 0);
            if (MV_CC_StartGrabbing(new_handle) == MV_OK) {
              std::lock_guard<std::mutex> lock(*(camera_mutexes_[cam_idx]));
              camera_handles_[cam_idx] = new_handle;
              RCLCPP_INFO(this->get_logger(), "相机%zu 重连成功", cam_idx);
              found = true;
              break;
            } else {
              MV_CC_CloseDevice(new_handle);
              MV_CC_DestroyHandle(&new_handle);
            }
          }
        }
      }
      if (found) break;
      RCLCPP_WARN(this->get_logger(), "相机%zu 重连等待...", cam_idx);
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }

  // ======================== 成员变量 ========================
  std::vector<void*> camera_handles_;
  std::vector<std::unique_ptr<std::mutex>> camera_mutexes_;
  std::vector<MV_IMAGE_BASIC_INFO> img_infos_;
  std::vector<MV_CC_PIXEL_CONVERT_PARAM> convert_params_;
  std::vector<sensor_msgs::msg::Image> image_msgs_;
  std::vector<sensor_msgs::msg::CameraInfo> camera_info_msgs_;
  std::vector<image_transport::CameraPublisher> camera_pubs_;
  std::vector<std::unique_ptr<camera_info_manager::CameraInfoManager>> info_managers_;
  std::vector<int> fail_counts_;
  std::vector<std::string> serial_numbers_;
  std::vector<double> fps_limits_;          // 新增：存储每台相机的帧率限制
  std::vector<std::thread> capture_threads_;
};

} // namespace hik_camera

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(hik_camera::HikDualCameraNode)


