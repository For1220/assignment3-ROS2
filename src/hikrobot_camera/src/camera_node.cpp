#include "hikrobot_camera/camera_node.hpp"
#include <cstring>
#include <chrono>
#include <functional>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <vector>
#include <opencv2/opencv.hpp>

namespace hikrobot_camera
{

CameraNode::CameraNode(const rclcpp::NodeOptions & options)
: Node("hikrobot_camera", options), is_connected_(false)
{
  this->declare_parameter<std::string>("topic_name", "image_raw");
  this->declare_parameter<std::string>("ip", "");
  this->declare_parameter<std::string>("serial_number", "");
  this->declare_parameter<std::string>("pixel_format", "BayerRG8");
  this->declare_parameter<double>("exposure_time", 5000.0);
  this->declare_parameter<double>("gain", 10.0);
  this->declare_parameter<double>("frame_rate", 30.0);

  std::string topic_name = this->get_parameter("topic_name").as_string();
  publisher_ = this->create_publisher<sensor_msgs::msg::Image>(topic_name, rclcpp::SensorDataQoS());

  param_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&CameraNode::OnParameterUpdate, this, std::placeholders::_1)
  );

  ConnectCamera();

  publish_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(15),
    std::bind(&CameraNode::PublishImage, this));

  reconnect_timer_ = this->create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&CameraNode::CheckConnection, this));
  
  last_frame_time_ = std::chrono::steady_clock::now();
}

CameraNode::~CameraNode()
{
  if (handle_) {
    MV_CC_StopGrabbing(handle_);
    MV_CC_CloseDevice(handle_);
    MV_CC_DestroyHandle(handle_);
    RCLCPP_INFO(this->get_logger(), "相机资源已释放。");
  }
}

void CameraNode::ConnectCamera()
{
  if (is_connected_) return;

  MV_CC_DEVICE_INFO_LIST stDeviceList;
  memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
  int nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
  if (nRet != MV_OK || stDeviceList.nDeviceNum == 0) {
    RCLCPP_WARN(this->get_logger(), "重连中：找不到相机设备！");
    return;
  }

  std::string target_ip = this->get_parameter("ip").as_string();
  std::string target_sn = this->get_parameter("serial_number").as_string();
  int selected_index = -1;

  for (unsigned int i = 0; i < stDeviceList.nDeviceNum; ++i) {
    MV_CC_DEVICE_INFO* pInfo = stDeviceList.pDeviceInfo[i];
    if (pInfo == nullptr) continue;

    std::string current_sn = "";
    std::string current_ip = "";

    if (pInfo->nTLayerType == MV_USB_DEVICE) {
      current_sn = std::string(reinterpret_cast<char*>(pInfo->SpecialInfo.stUsb3VInfo.chSerialNumber));
    } else if (pInfo->nTLayerType == MV_GIGE_DEVICE) {
      current_sn = std::string(reinterpret_cast<char*>(pInfo->SpecialInfo.stGigEInfo.chSerialNumber));
      unsigned int ip_int = pInfo->SpecialInfo.stGigEInfo.nCurrentIp;
      char ip_str[32];
      snprintf(ip_str, sizeof(ip_str), "%d.%d.%d.%d",
               (ip_int >> 24) & 0xFF, (ip_int >> 16) & 0xFF,
               (ip_int >> 8) & 0xFF, ip_int & 0xFF);
      current_ip = std::string(ip_str);
    }

    bool ip_match = (!target_ip.empty() && target_ip == current_ip);
    bool sn_match = (!target_sn.empty() && target_sn == current_sn);

    if (ip_match || sn_match) {
      selected_index = i;
      RCLCPP_INFO(this->get_logger(), "成功匹配到设备: IP=%s, SN=%s", current_ip.c_str(), current_sn.c_str());
      break;
    }
  }

  if (selected_index == -1) {
    if (target_ip.empty() && target_sn.empty()) {
      RCLCPP_INFO(this->get_logger(), "未指定 IP 或序列号，默认连接第一个设备。");
      selected_index = 0;
    } else {
      RCLCPP_ERROR(this->get_logger(), "未找到匹配 IP(%s) 或 序列号(%s) 的相机！", target_ip.c_str(), target_sn.c_str());
      return;
    }
  }

  MV_CC_CreateHandle(&handle_, stDeviceList.pDeviceInfo[selected_index]);
  nRet = MV_CC_OpenDevice(handle_);
  if (nRet != MV_OK) {
    RCLCPP_ERROR(this->get_logger(), "打开设备失败，错误码: %x。", nRet);
    MV_CC_DestroyHandle(handle_); handle_ = nullptr; return;
  }

  MV_CC_RegisterImageCallBackEx(handle_, ImageCallback, this);
  MV_CC_StartGrabbing(handle_);
  ApplyParamsToCamera();
  is_connected_ = true;
  last_frame_time_ = std::chrono::steady_clock::now();

  // 获取并保存当前相机的像素格式
  MVCC_ENUMVALUE stEnumValue = {};
  int nEnumRet = MV_CC_GetEnumValue(handle_, "PixelFormat", &stEnumValue);
  if (nEnumRet == MV_OK) {
    current_pixel_format_ = stEnumValue.nCurValue;
    RCLCPP_INFO(this->get_logger(), "当前生效像素格式枚举值: %u", current_pixel_format_);
    
    RCLCPP_INFO(this->get_logger(), "相机支持 %d 种像素格式:", stEnumValue.nSupportedNum);
    for (unsigned int i = 0; i < stEnumValue.nSupportedNum; i++) {
      RCLCPP_INFO(this->get_logger(), "  支持格式索引 %d (枚举值): %d", i, stEnumValue.nSupportValue[i]);
    }
  } else {
    RCLCPP_WARN(this->get_logger(), "获取像素格式列表失败，错误码: %x", nEnumRet);
  }
  RCLCPP_INFO(this->get_logger(), "相机连接成功，开始采集！");
}

void CameraNode::ApplyParamsToCamera()
{
  if (!handle_) return;
  MV_CC_SetEnumValue(handle_, "ExposureAuto", 0);
  MV_CC_SetFloatValue(handle_, "ExposureTime", this->get_parameter("exposure_time").as_double());
  MV_CC_SetEnumValue(handle_, "GainAuto", 0);
  MV_CC_SetFloatValue(handle_, "Gain", this->get_parameter("gain").as_double());
  MV_CC_SetBoolValue(handle_, "AcquisitionFrameRateEnable", true);
  MV_CC_SetFloatValue(handle_, "AcquisitionFrameRate", this->get_parameter("frame_rate").as_double());
  RCLCPP_INFO(this->get_logger(), "已恢复相机参数 (曝光/增益/帧率)");
}

void CameraNode::CheckConnection()
{
  if (is_connected_ && handle_) {
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - last_frame_time_).count();
    if (elapsed > 3) {
      RCLCPP_WARN(this->get_logger(), "超过3秒未收到图像，正在尝试重连...");
      MV_CC_StopGrabbing(handle_); MV_CC_CloseDevice(handle_); MV_CC_DestroyHandle(handle_);
      handle_ = nullptr; is_connected_ = false;
    }
  }
  if (!is_connected_) ConnectCamera();
}

rcl_interfaces::msg::SetParametersResult CameraNode::OnParameterUpdate(
    const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto & param : parameters) {
    if (!handle_) {
      RCLCPP_WARN(this->get_logger(), "无相机连接，仅更新参数 [%s]", param.get_name().c_str());
      continue; 
    }

    if (param.get_name() == "exposure_time") {
      double exp_val = param.as_double();
      if (exp_val < 10.0 || exp_val > 1000000.0) {
        result.successful = false; result.reason = "曝光参数超出范围 (10.0 - 1000000.0)，拒绝设置"; return result;
      }
      MV_CC_SetEnumValue(handle_, "ExposureAuto", 0);
      int ret = MV_CC_SetFloatValue(handle_, "ExposureTime", param.as_double());
      if (ret != MV_OK) { result.successful = false; result.reason = "曝光设置失败，错误码: " + std::to_string(ret); }
    } else if (param.get_name() == "gain") {
      double gain_val = param.as_double();
      if (gain_val < 0.0 || gain_val > 100.0) {
        result.successful = false; result.reason = "增益参数超出范围 (0.0 - 100.0)，拒绝设置"; return result;
      }
      MV_CC_SetEnumValue(handle_, "GainAuto", 0);
      int ret = MV_CC_SetFloatValue(handle_, "Gain", param.as_double());
      if (ret != MV_OK) { result.successful = false; result.reason = "增益设置失败，错误码: " + std::to_string(ret); }
    } else if (param.get_name() == "frame_rate") {
      MV_CC_SetBoolValue(handle_, "AcquisitionFrameRateEnable", true);
      int ret = MV_CC_SetFloatValue(handle_, "AcquisitionFrameRate", param.as_double());
      if (ret != MV_OK) { result.successful = false; result.reason = "帧率设置失败，错误码: " + std::to_string(ret); }
    } else if (param.get_name() == "pixel_format") {
      MV_CC_StopGrabbing(handle_);
      int ret = MV_CC_SetEnumValueByString(handle_, "PixelFormat", param.as_string().c_str());
      if (ret == MV_OK) {
          // 切换成功后，更新当前像素格式记录
          MVCC_ENUMVALUE stEnumValue = {};
          if (MV_CC_GetEnumValue(handle_, "PixelFormat", &stEnumValue) == MV_OK) {
              current_pixel_format_ = stEnumValue.nCurValue;
          }
      }
      MV_CC_StartGrabbing(handle_);
      if (ret != MV_OK) { result.successful = false; result.reason = "像素格式切换失败，SDK错误码: " + std::to_string(ret); }
    }
  }
  return result;
}

void __stdcall CameraNode::ImageCallback(unsigned char * pData, MV_FRAME_OUT_INFO_EX * pFrameInfo, void * pUser)
{
  CameraNode * node = static_cast<CameraNode *>(pUser);
  if (pFrameInfo && pData) node->HandleImage(pData, pFrameInfo);
}

void CameraNode::HandleImage(unsigned char * pData, MV_FRAME_OUT_INFO_EX * pFrameInfo)
{
  std::lock_guard<std::mutex> lock(queue_mutex_);
  last_frame_time_ = std::chrono::steady_clock::now();
  std::vector<unsigned char> img_data(pData, pData + pFrameInfo->nFrameLen);
  frame_queue_.push(std::make_pair(img_data, *pFrameInfo));
  if (frame_queue_.size() > 5) frame_queue_.pop();
}

void CameraNode::PublishImage()
{
  std::lock_guard<std::mutex> lock(queue_mutex_);
  if (frame_queue_.empty()) return;

  auto & frame_pair = frame_queue_.front();
  auto & data = frame_pair.first;
  auto & info = frame_pair.second;

  // 统一使用 SDK 的万能转换接口，将各种格式（含Packed和10/12位）转为 BGR8
  MV_CC_PIXEL_CONVERT_PARAM stConvertParam = { };
  stConvertParam.nWidth = info.nWidth;
  stConvertParam.nHeight = info.nHeight;
  stConvertParam.pSrcData = data.data();
  stConvertParam.nSrcDataLen = data.size();
  stConvertParam.enSrcPixelType = static_cast<MvGvspPixelType>(current_pixel_format_);
  stConvertParam.enDstPixelType = PixelType_Gvsp_BGR8_Packed;

  // 分配目标缓冲区（BGR8 = 3字节/像素）
  unsigned int dst_buffer_size = info.nWidth * info.nHeight * 3;
  std::vector<unsigned char> bgr_data(dst_buffer_size);
  stConvertParam.pDstBuffer = bgr_data.data();
  stConvertParam.nDstBufferSize = dst_buffer_size;
  stConvertParam.nDstLen = dst_buffer_size;

  int ret = MV_CC_ConvertPixelType(handle_, &stConvertParam);
  if (ret != MV_OK) {
    RCLCPP_ERROR(this->get_logger(), "SDK像素格式转换失败，错误码: %x", ret);
    frame_queue_.pop(); return;
  }

  // 此时 bgr_data 里的数据已经是标准的 BGR8 彩色图像
  cv::Mat bgr_img(info.nHeight, info.nWidth, CV_8UC3, bgr_data.data());
  cv::Mat final_img;

  // 防止 DDS 溢出，强制缩放到 640x480
  if (bgr_img.cols > 640 || bgr_img.rows > 480) {
    cv::resize(bgr_img, final_img, cv::Size(640, 480));
  } else {
    final_img = bgr_img;
  }

  auto msg = sensor_msgs::msg::Image();
  msg.header.stamp = this->now();
  msg.header.frame_id = "camera_link";
  msg.height = final_img.rows; // 480
  msg.width = final_img.cols;  // 640
  msg.encoding = "bgr8";
  msg.step = msg.width * 3;    // 1920
  msg.data.assign(final_img.data, final_img.data + final_img.total() * final_img.elemSize());

  publisher_->publish(msg);
  frame_queue_.pop();

  static int frame_count = 0;
  static auto last_time = std::chrono::steady_clock::now();
  frame_count++;
  auto now_time = std::chrono::steady_clock::now();
  if (std::chrono::duration_cast<std::chrono::seconds>(now_time - last_time).count() >= 1) {
    double actual_fps = frame_count / std::chrono::duration_cast<std::chrono::seconds>(now_time - last_time).count();
    double set_fps = this->get_parameter("frame_rate").as_double();
    RCLCPP_INFO(this->get_logger(), "设定帧率: %.1f FPS, 实际接收帧率: %.1f FPS", set_fps, actual_fps);
    frame_count = 0;
    last_time = now_time;
  }
}

}  // namespace hikrobot_camera