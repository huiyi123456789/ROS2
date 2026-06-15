# ROS2 机器人仿真与控制平台

## 项目概述

本项目是一个完整的 ROS2 + Gazebo 机器人仿真与控制平台，包含四个核心模块：

1. **手机视频采集与传输** — 利用手机浏览器拍摄并记录机器人移动轨迹，将视频传输至服务器。
2. **Gazebo 仿真环境** — 教室场景仿真，支持机器人/传感器动态增删，集成激光雷达、摄像头、IMU 等传感器。
3. **机器人描述与可视化** — 基于 URDF/Xacro 的机器人模型，支持 RViz2 三维可视化。
4. **Qt6 机器人控制面板** — 图形化远程控制界面，支持键盘/鼠标操控、地图查看、摄像头画面显示、实体生成管理。

## 目录结构

```
ROS2/
├── web/                           # Web端视频采集页面
│   ├── index.html                 # 摄像头采集、录制、上传页面
│   ├── style.css                  # 移动端优先响应式样式
│   ├── server.py                  # 视频上传服务端
│   └── uploads/                   # 上传视频存储目录
├── scripts/                       # 辅助脚本
│   └── start_test_streams.sh      # 启动 ffmpeg RTSP 测试视频流
├── src/
│   ├── robot_proj/                # Gazebo 仿真环境包
│   │   ├── launch/                # 仿真启动文件
│   │   │   └── classroom_sim.launch.py
│   │   ├── worlds/                # SDF 世界文件
│   │   │   └── classroom.world    # 教室场景（含墙体、门窗、课桌椅等）
│   │   ├── models/                # SDF 模型
│   │   │   ├── custom_robot/      # 差速驱动机器人（激光雷达+摄像头+IMU）
│   │   │   └── classroom_furniture/  # 课桌模型
│   │   └── scripts/               # Python 管理工具
│   │       ├── spawn_robot.py     # 机器人动态生成/删除
│   │       ├── manage_sensors.py  # 传感器管理
│   │       └── demo_scenarios.py  # 预设场景运行器
│   ├── robot_description/         # 机器人描述与可视化包
│   │   ├── launch/
│   │   │   └── rviz.launch.py     # RViz2 可视化启动文件
│   │   ├── urdf/
│   │   │   └── robot.urdf.xacro   # Xacro 机器人 URDF 模型
│   │   └── config/
│   │       └── display.rviz       # RViz2 配置文件
│   └── robot_controller/          # Qt6 机器人控制面板
│       ├── launch/
│       │   └── control_panel.launch.py
│       └── src/                   # C++ 源文件
│           ├── main.cpp           # 程序入口
│           ├── MainWindow.h/cpp   # 主窗口（布局编排、键盘事件）
│           ├── RobotController.h/cpp        # cmd_vel 运动控制发布器
│           ├── RobotControlWidget.h/cpp     # 机器人选择与操控界面
│           ├── MapWidget.h/cpp              # 占用网格地图显示
│           ├── RosCameraWidget.h/cpp        # ROS 图像话题可视化
│           ├── SpawnWidget.h/cpp            # 实体生成/删除管理
│           └── VideoWidget.h/cpp            # RTSP 视频播放器
├── build/                         # 构建目录
├── install/                       # 安装目录
└── log/                           # 日志目录
```

## 快速开始

### 1. 启动视频采集服务器

```bash
python3 web/server.py
```

手机浏览器访问 `http://<服务器IP>:8081/` 即可使用摄像头采集、录制和上传功能。

### 2. 构建项目

```bash
colcon build --symlink-install
source install/setup.bash
```

### 3. 启动 Gazebo 仿真环境

```bash
ros2 launch robot_proj classroom_sim.launch.py
```

这会启动包含教室场景的 Gazebo 仿真，并生成初始机器人 `robot_01`。

### 4. 启动机器人控制面板

```bash
ros2 launch robot_controller control_panel.launch.py
```

打开 Qt6 图形化控制面板，可以：
- 选择并控制机器人移动（WASD/方向键/按钮）
- 查看机器人前后摄像头画面
- 查看激光雷达构建的地图
- 动态生成/删除机器人和传感器
- 运行预设场景（教室导航、多机器人编队、传感器套件）

### 5. 启动 RViz 可视化

```bash
ros2 launch robot_description rviz.launch.py
```

在 RViz2 中查看机器人三维模型和传感器数据。

### 6. 辅助工具

```bash
# 启动 RTSP 测试视频流（用于测试 VideoWidget）
./scripts/start_test_streams.sh

# 机器人管理命令行工具
python3 src/robot_proj/scripts/spawn_robot.py --demo

# 传感器管理命令行工具
python3 src/robot_proj/scripts/manage_sensors.py --demo

# 预设场景运行
python3 src/robot_proj/scripts/demo_scenarios.py <1-4>
```

## 功能说明

### Web端视频采集
- **摄像头预览** — 自动请求后置摄像头权限，显示实时画面
- **录制视频** — 点击"开始录制"录制画面，点击"停止录制"生成视频文件
- **视频上传** — 将录制的视频上传到服务器
- **本地下载** — 将视频保存到手机本地

### 控制面板
- **机器人运动控制** — 前进/后退/左转/右转/停止，可控线速度和角速度
- **键盘操控** — WASD 或方向键直接控制机器人
- **摄像头画面** — 实时显示机器人前后的 ROS 图像话题
- **地图查看** — 显示 `/map` 话题的占用网格地图
- **实体管理** — 动态生成/删除机器人，支持添加 2D LiDAR、3D LiDAR、RGB 摄像头、深度摄像头、IMU 传感器
- **预设场景** — 一键部署教室导航、多机器人编队等场景

### 仿真环境
- **教室场景** — 完整的教室仿真环境，含墙体、门窗、黑板、时钟、课桌椅、垃圾桶
- **差速驱动机器人** — 集成激光雷达（360°扫描）、RGB 摄像头、IMU 的移动机器人
- **物理引擎** — 基于 ODE 的物理仿真，支持碰撞检测
- **传感器仿真** — 2D/3D 激光雷达、RGB/深度摄像头、IMU，支持噪声模拟

## 依赖

- Python 3
- ROS2 Humble
- Gazebo Classic + `ros-humble-gazebo-ros-pkgs`
- Qt6（Widgets、Multimedia、MultimediaWidgets）
- FFmpeg（可选，用于 RTSP 测试流）
