# 第四次作业：ROS 2 红蓝色块实时识别（C++）

## 一、作业目标

基于 **ROS 2 Humble + C++ + OpenCV** 完成实时红、蓝色块识别：

- 创建两个 ROS 2 节点，分别负责摄像头图像采集与颜色识别。
- 使用 **HSV 颜色阈值法** 识别红色色块，使用 **BGR 通道差值法** 识别蓝色色块。
- 显示原始画面、红色二值图、蓝色二值图和带检测框的结果。
- 使用笔记本内置摄像头进行实时测试。

## 二、运行环境

- Windows + WSL2（Ubuntu 22.04.5 LTS）
- ROS 2 Humble
- C++17、OpenCV、`cv_bridge`、`rclcpp`
- Windows 端 FFmpeg（用于向 WSL 转发笔记本内置摄像头画面）

> 由于 WSL 内没有 `/dev/video0`，本次通过 Windows FFmpeg 提供实时 MJPEG 视频流；图像发布和处理两个 ROS 2 节点均使用 **C++** 实现。

## 三、项目结构

```text
rm_color_cpp/
├── CMakeLists.txt
├── package.xml
├── README.md
└── src/
    ├── camera_node.cpp          # 摄像头图像采集及 ROS 2 发布
    └── color_detector_node.cpp  # 图像订阅、红蓝色块识别与显示
```

## 四、实现原理

### 1. 两个节点之间的通信

```text
Windows 内置摄像头
        │
        ▼
   FFmpeg 实时视频流
        │  HTTP MJPEG
        ▼
   camera_node（C++）
        │  发布 /camera/image_raw
        ▼
color_detector_node（C++）
        ├── 红色：HSV 双区间阈值分割
        ├── 蓝色：BGR 通道差值阈值分割
        └── 二值图、轮廓筛选、检测框
```

`camera_node` 使用 OpenCV 的 `cv::VideoCapture` 读取视频流，并通过 `cv_bridge` 转换为 ROS 2 的 `sensor_msgs/msg/Image` 消息；`color_detector_node` 订阅图像话题后进行识别。

### 2. 红色色块识别：HSV 阈值法

将 BGR 图像转换为 HSV 图像。OpenCV 中 H 的范围为 `0~179`，红色色相位于色相环两端，因此分别提取两个区间后合并：

- H：`0~10` 或 `170~179`
- S：`100~255`
- V：`65~255`

使用 `cv::inRange()` 生成红色二值掩膜。

### 3. 蓝色色块识别：BGR 通道差值法

直接比较原始图像的 B（蓝）通道与 R（红）通道：

- `B - R > 60`
- `B > 100`

同时满足条件的像素被保留为白色，其余区域为黑色。该方法也可能识别蓝青色区域。

### 4. 后处理

对红蓝二值图分别进行 3×3 核的开、闭运算，以减少小噪点并填补局部空隙。随后通过 `findContours()` 查找轮廓，过滤面积小于 `400` 像素的区域，并在原始画面中绘制外接矩形及 `RED` / `BLUE` 标签。

## 五、编译

将 `rm_color_cpp` 放到 ROS 2 工作空间的 `src/` 目录下。工作空间示例为 `~/vision_project/ros2_ws`。

```bash
source /opt/ros/humble/setup.bash
cd ~/vision_project/ros2_ws
colcon build --symlink-install --packages-select rm_color_cpp
source install/setup.bash
ros2 pkg executables rm_color_cpp
```

正常情况下可看到 `camera_node` 和 `color_detector_node` 两个可执行程序。

## 六、运行（笔记本内置摄像头 + WSL2）

### 1. Windows PowerShell：启动摄像头视频流

先安装 FFmpeg，并用 `ffmpeg -list_devices true -f dshow -i dummy` 查看实际摄像头名称。本次测试摄像头为 `USB2.0 VGA UVC WebCam`。

```powershell
ffmpeg -rtbufsize 100M -f dshow -i "video=USB2.0 VGA UVC WebCam" -an -vf "fps=15,scale=640:480" -c:v mjpeg -q:v 5 -f mpjpeg -content_type "multipart/x-mixed-replace;boundary=ffmpeg" -listen 1 "http://0.0.0.0:8765/video"
```

保持 PowerShell 运行。该视频流仅用于本机可信环境下的作业测试，结束后应关闭。

### 2. WSL Ubuntu 终端 1：启动图像发布节点

```bash
source ~/vision_project/ros2_ws/install/setup.bash
WIN_IP=$(ip route show default | awk '{print $3; exit}')
echo "$WIN_IP"
ros2 run rm_color_cpp camera_node --ros-args -p source:="http://${WIN_IP}:8765/video"
```

以上适用于常见的 WSL NAT 网络；若启用镜像网络模式，可以尝试将地址改为 `127.0.0.1`。

### 3. WSL Ubuntu 终端 2：启动颜色识别节点

```bash
source ~/vision_project/ros2_ws/install/setup.bash
ros2 run rm_color_cpp color_detector_node
```

### 4. 检查节点与话题

在第三个 Ubuntu 终端中执行：

```bash
source ~/vision_project/ros2_ws/install/setup.bash
ros2 node list
ros2 topic list
ros2 topic info /camera/image_raw -v
```

主要话题如下：

| 话题 | 内容 |
| --- | --- |
| `/camera/image_raw` | 摄像头原始图像（BGR8） |
| `/color/red_mask` | 红色二值图（MONO8） |
| `/color/blue_mask` | 蓝色二值图（MONO8） |
| `/color/detected` | 带识别框的画面（BGR8） |

## 七、识别结果

程序运行后会显示四个窗口：

| 窗口 | 显示内容 |
| --- | --- |
| `Original` | 摄像头实时画面 |
| `Threshold - Red HSV` | 红色区域为白色，其余为黑色 |
| `Threshold - Blue BGR` | 蓝色区域为白色，其余为黑色 |
| `Detected` | 在画面中绘制红蓝区域外接框及标签 |

测试时分别将红色和蓝色图片放在摄像头前，观察对应二值图及框选结果。建议把实际运行截图保存到仓库的 `screenshots/` 目录，用于展示作业效果。

## 八、总结

本次作业通过 ROS 2 的发布/订阅机制完成图像采集和处理的节点分离，并使用 OpenCV 的两种颜色分割方法实现红蓝色块实时识别。HSV 法适合按色相提取红色，BGR 通道差值法可以直接利用通道强度差提取蓝色。阈值受环境光、相机自动曝光和白平衡影响，实际应用中可以根据环境进一步调整。
