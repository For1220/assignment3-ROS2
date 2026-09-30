
## 环境配置
- Ubuntu 22.04 LTS
- ROS2 Humble
- SDK:海康 MVS V5.1.0(Linux x86_64)
- GCC 11+(支持 C++17)
- colcon-core:0.21.3

## 安装，编译和运行
### ROS和系统依赖安装
`package.xml` 声明依赖：
```bash
  <depend>rclcpp</depend>
  <depend>sensor_msgs</depend>
```
然后在终端的根目录上使用 `rosdepc install --from-paths src --ignore-src -r -y`
因为输入 `rosdep` 通常会因为网络问题无法连接

然后进入海康机器人官网络下载 Linux x86_64 版本的 MVS SDK 安装包
利用 `sudo dpkg -i (刚刚的安装包全名.deb)`，其中默认路径为 `/opt/MVS`
接着在 `CMakeLists.txt` 中配置正确的头文件路径 `/opt/MVS/include` 和库路径 `/opt/MVS/lib/64`

### 编译
``` bash
cd ~/ros2_ws
colcon build --packages-select hikrobot_camera
```
成功编译后会显示 `Summary: 1 package finished`
### 运行
因为用 `sudo` 去运行会有一个提权的过程，后续在第二个终端运行 `RViz2` 去读 `/image_raw` 的时候会因为不平级而读不到相机画面，需要提前配置 `udev` 规则。
- 终端 `sudo nano /etc/udev/rules.d/99-mvs-camera.rules`
- 文本框内写入： `SUBSYSTEM=="usb", ATTRS{idVendor}=="2bdf", MODE="0666", GROUP="plugdev"`
- 终端 `sudo udevadm control --reload-rules && sudo udevadm trigger`
重新插拔 USB 线即可实现。

接着我们因为禁用了全局 `MVS` 的库路径来防止动态写入(下文还有提及为啥要这样做)，需要在每一个新终端里面临时注入库路径：
```bash
export LD_LIBRARY_PATH=/opt/MVS/lib/64:/opt/MVS/bin:$LD_LIBRARY_PATH && source /opt/ros/humble/setup.zsh && source ~/ros2_ws/install/setup.zsh && ros2 launch hikrobot_camera camera.launch.py
```

然后再启动节点：
```bash
source /opt/ros/humble/setup.zsh
source install/setup.zsh
ros2 launch hikrobot_camera camera.launch.py
```
然后你就能看到提示 `相机连接成功，开始采集` 的提示词，证明启动成功。

## 基本参数配置
- `topic_name`：默认值 `image_raw`，图像发布话题，可动态配置
- `exposure_time`：默认值 `5000.0`，曝光时间，可手动调整
- `gain`：默认值 `10.0`，增益，可手动调整
- `frame_rate`：默认值 `30.0`，设定理论帧率，可手动调整
- `pixel_format`：默认值 `BayerRGB`，像素格式，可以手动调整

支持验证和手动调整：
```bash
ros2 param list /hikrobot_camera
ros2 param get /hikrobot_camera exposure_time
ros2 param set /hikrobot_camera exposure_time 10000.0
```
- `.cpp`中添加了对于曝光和增益的范围校验，若不合法的参数调教将返回诸如以下的文字：`"曝光参数超出范围 (10.0 - 1000000.0)，拒绝设置"`
- `SDK` 返回值校验：设置失败会返回明确的 `SDK` 错误码
- `pixel_format` 校验：首先，成功连接相机时会打印合法的相机硬件支持的格式枚举值列表(需要SDK)，接着，如果 `set` 一个错误的格式，会返回 `像素格式切换失败`。
- 自动模式关闭
- 像素模式实现暂停采集到设置格式到恢复采集的流程。

## RViz2 启动
```bash
unset LD_LIBRARY_PATH
source /opt/ros/humble/setup.zsh
export QT_QPA_PLATFORM=xcb
rviz2
```

## 已知环境问题与排查说明 （本节利用 ds 整理）
本节点核心驱动功能（设备枚举、打开相机、图像采集、发布话题）已完整实现。在运行过程中，由于 Ubuntu 22.04 + ROS 2 Humble 的底层 DDS 通信机制限制，存在以下环境问题：
1. **Qt 库冲突**：由于海康 MVS SDK 自带旧版 Qt 库（`libQt5Gui.so.5`），导致 RViz2 和 rqt_image_view 启动时崩溃。建议使用命令行工具进行验证。
2. **大图像传输限制（sequence size exceeds remaining buffer）**：由于相机输出 1440x1080 的高分辨率图像（单帧约 1.5MB），超出了 ROS 2 默认 DDS 的 UDP 传输缓冲区大小。这属于底层中间件限制，节点仍在正常运行。
3. **推荐验证方式**：建议使用 `ros2 topic info /image_raw -v` 查看话题状态。如需实际查看图像，建议在 `camera_node.cpp` 中降低分辨率，或在参数中限制图像大小。
4. **USB 设备占用**：若出现 `80000203` 错误，请执行 `sudo pkill -f MVS` 并拔插 USB 线。

## 解决办法
1. 对于 `Qt` 库的冲突，经过学长的帮助已经成功定位是 `MVS` 启用了全局动态库输入，删除 `MVS` 配置文件对于系统配置的写入即可。