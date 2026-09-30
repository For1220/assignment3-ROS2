#ifndef HIKROBOT_CAMERA_CAMERA_NODE_HPP
#define HIKROBOT_CAMERA_CAMERA_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "MvCameraControl.h"
#include <mutex>
#include <queue>
#include <vector>
#include <utility>
#include <chrono>

namespace hikrobot_camera
{
class CameraNode : public rclcpp::Node
{
public:
  explicit CameraNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~CameraNode() override;

private:
  unsigned int current_pixel_format_ = 0; // 记录当前生效的像素格式枚举值
  // 核心功能函数
  void ConnectCamera();       
  void ApplyParamsToCamera(); 
  void CheckConnection();     
  
  static void __stdcall ImageCallback(unsigned char * pData, MV_FRAME_OUT_INFO_EX * pFrameInfo, void * pUser);
  void HandleImage(unsigned char * pData, MV_FRAME_OUT_INFO_EX * pFrameInfo);
  void PublishImage();
  rcl_interfaces::msg::SetParametersResult OnParameterUpdate(const std::vector<rclcpp::Parameter> & parameters);

  // ROS成员
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr reconnect_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
  
  // 相机与状态控制
  void * handle_ = nullptr;
  std::mutex queue_mutex_;
  std::queue<std::pair<std::vector<unsigned char>, MV_FRAME_OUT_INFO_EX>> frame_queue_;
  
  std::chrono::steady_clock::time_point last_frame_time_;
  bool is_connected_ = false;
};
}  // namespace hikrobot_camera

#endif  // HIKROBOT_CAMERA_CAMERA_NODE_HPP