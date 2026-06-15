# ROS2 机器人仿真与控制平台

## 项目概述

本项目是一个完整的 ROS2 + Gazebo 机器人仿真与控制平台，包含四个核心模块：

1. **手机视频采集与传输** — 利用手机浏览器拍摄并记录机器人移动轨迹，将视频传输至服务器。
2. **Gazebo 仿真环境** — 教室场景仿真，基于 **TurtleBot3 Waffle** 机器人，支持动态增删机器人/传感器。
3. **机器人描述与可视化** — 基于 TurtleBot3 URDF 的机器人模型，支持 RViz2 三维可视化。
4. **Qt6 机器人控制面板** — 图形化远程控制界面，支持键盘/鼠标操控、地图查看、摄像头画面、实体管理。

## 机器人模型

采用官方 **[TurtleBot3 Waffle](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)** 机器人：

| 特性 | 参数 |
|------|------|
| 驱动方式 | 差速驱动（两轮独立驱动 + 两后万向轮） |
| 轮径/轮距 | 0.066m / 0.287m |
| 激光雷达 | 360° LDS-01，5Hz，0.12–3.5m（默认关闭） |
| IMU | 三轴加速度计 + 陀螺仪，200Hz |
| RGB 摄像头 | Intel RealSense R200，1920×1080，30fps |
| 最大扭矩 | 20 N·m |
| 车轮摩擦系数 | mu ≈ ∞（极端抓地力） |
| 地面接触刚度 | kp=1e5, kd=1（强接触约束，消除弹跳） |

> 激光雷达默认关闭（`always_on=0`），仅在订阅话题时自动开启，节省仿真计算资源。

## 目录结构

```
ROS2/
├── web/                            # Web端视频采集页面
│   ├── index.html                  # 摄像头采集、录制、上传页面
│   ├── style.css                   # 移动端优先响应式样式
│   ├── server.py                   # 视频上传服务端
│   └── uploads/                    # 上传视频存储目录
├── scripts/                        # 辅助脚本
│   └── start_test_streams.sh       # ffmpeg RTSP 测试视频流
├── src/
│   ├── robot_proj/                 # Gazebo 仿真环境包
│   │   ├── launch/
│   │   │   └── classroom_sim.launch.py
│   │   ├── worlds/
│   │   │   └── classroom.world     # 教室场景（墙/门/窗/桌椅/黑板等）
│   │   ├── models/
│   │   │   ├── turtlebot3_waffle/  # TurtleBot3 Waffle SDF 模型
│   │   │   └── classroom_furniture/
│   │   └── scripts/
│   │       ├── spawn_robot.py
│   │       ├── manage_sensors.py
│   │       └── demo_scenarios.py
│   ├── robot_description/          # RViz 可视化
│   │   ├── launch/rviz.launch.py
│   │   ├── urdf/robot.urdf.xacro   # TurtleBot3 Waffle URDF
│   │   └── config/display.rviz
│   └── robot_controller/           # Qt6 控制面板
│       ├── launch/control_panel.launch.py
│       └── src/                    # C++ 源文件（7 个 Widget 类）
├── build/ / install/ / log/
```

## 快速开始

### 0. 安装依赖

```bash
sudo apt install ros-humble-turtlebot3-gazebo ros-humble-turtlebot3-description
```

### 1. 启动视频采集服务器

```bash
python3 web/server.py
# 手机浏览器访问 http://<服务器IP>:8081/
```

### 2. 构建项目

```bash
colcon build --symlink-install
source install/setup.bash
```

### 3. 启动 Gazebo 仿真

```bash
ros2 launch robot_proj classroom_sim.launch.py
```

启动教室场景 + 初始 TurtleBot3 机器人 `robot_01`。

### 4. 启动控制面板

```bash
ros2 launch robot_controller control_panel.launch.py
```

Qt6 图形化控制面板，支持：
- 键盘/按钮操控机器人（WASD/方向键）
- 查看机器人 RGB 摄像头实时画面
- 查看激光雷达地图
- 动态生成/删除机器人
- 添加传感器（2D/3D LiDAR、RGB/深度相机、IMU）
- 运行 4 个预设场景

### 5. 启动 RViz 可视化

```bash
ros2 launch robot_description rviz.launch.py
```

## ROS2 话题对照

| 话题 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `/robot_01/cmd_vel` | Twist | 控制→仿真 | 速度指令 |
| `/robot_01/odom` | Odometry | 仿真→控制 | 里程计 |
| `/robot_01/scan` | LaserScan | 仿真→控制 | 激光扫描 |
| `/robot_01/imu` | Imu | 仿真→控制 | IMU 数据 |
| `/robot_01/camera/image_raw` | Image | 仿真→控制 | RGB 图像 |
| `/robot_01/joint_states` | JointState | 仿真→控制 | 关节状态 |

## 抓地力与物理参数

**地面**（classroom.world 瓷砖地板）：
- 摩擦系数：mu=1.0, mu2=1.0
- 接触参数：kp=1e5, kd=1, max_vel=0.01, min_depth=0.001

**车轮**（TurtleBot3 橡胶轮胎）：
- 摩擦系数：mu≈∞ (1e5)，零滑移
- 接触刚度：kp=1e5, kd=1（与地面匹配）

两人参数组合确保机器人行驶时不会打滑、弹跳或侧滑。

## 依赖

- Python 3
- ROS2 Humble
- Gazebo Classic + `ros-humble-gazebo-ros-pkgs`
- `ros-humble-turtlebot3-gazebo`
- `ros-humble-turtlebot3-description`
- Qt6（Widgets、Multimedia、MultimediaWidgets）
- FFmpeg（可选，RTSP 测试流）
