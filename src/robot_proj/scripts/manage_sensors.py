#!/usr/bin/env python3
"""
传感器动态增删管理工具
支持: 2D LiDAR | 3D LiDAR | RGB Camera | Depth Camera | IMU

每种传感器以独立 Model 的方式通过 /spawn_entity 添加到仿真中。
传感器 SDF 字符串由 build_*_sdf 系列方法动态生成，支持完全参数化。

用法:
  python3 manage_sensors.py              # 交互模式
  python3 manage_sensors.py --demo       # 演示模式
"""

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity, DeleteEntity, GetModelList
import sys


class SensorManager(Node):
    """传感器生命周期管理器

    支持传感器类型:
    - 2D LiDAR: 水平面 360° 激光扫描仪
    - 3D LiDAR: Velodyne VLP-16 风格多层激光雷达
    - RGB Camera: 标准彩色摄像头
    - Depth Camera: 深度相机
    - IMU: 惯性测量单元
    """

    def __init__(self):
        super().__init__('sensor_manager')
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.delete_client = self.create_client(DeleteEntity, '/delete_entity')
        self.model_list_client = self.create_client(GetModelList, '/get_model_list')

        self.spawn_client.wait_for_service()
        self.delete_client.wait_for_service()
        self.model_list_client.wait_for_service()

        self.get_logger().info('SensorManager 就绪')

    # ============================
    #  预定义传感器 SDF 模板
    # ============================

    @staticmethod
    def build_lidar_2d_sdf(name, namespace='', samples=360,
                           min_angle=-3.14159, max_angle=3.14159,
                           range_min=0.12, range_max=10.0,
                           x=0, y=0, z=0.15):
        """构建 2D LiDAR 的 SDF 字符串

        Args:
            name: 传感器名称
            namespace: ROS 命名空间
            samples: 水平采样点数（默认 360，即 1° 分辨率）
            min_angle/max_angle: 扫描角度范围（弧度）
            range_min/max: 测距范围（米）
            x, y, z: 世界坐标位置
        """
        ns_tag = f'<namespace>{namespace}</namespace>' if namespace else ''
        return f'''<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="{name}">
    <pose>{x} {y} {z} 0 0 0</pose>
    <link name="{name}_link">
      <visual name="visual">
        <geometry><cylinder><radius>0.04</radius><length>0.03</length></cylinder></geometry>
        <material><ambient>1 0 0 1</ambient></material>
      </visual>
      <sensor name="lidar" type="ray">
        <always_on>1</always_on>
        <visualize>true</visualize>
        <update_rate>10</update_rate>
        <ray>
          <scan>
            <horizontal>
              <samples>{samples}</samples>
              <resolution>1</resolution>
              <min_angle>{min_angle}</min_angle>
              <max_angle>{max_angle}</max_angle>
            </horizontal>
          </scan>
          <range>
            <min>{range_min}</min>
            <max>{range_max}</max>
            <resolution>0.01</resolution>
          </range>
          <noise>
            <type>gaussian</type>
            <mean>0.0</mean>
            <stddev>0.01</stddev>
          </noise>
        </ray>
        <plugin name="lidar_plugin" filename="libgazebo_ros_ray_sensor.so">
          <ros>{ns_tag}<argument>~/out:=scan</argument></ros>
          <output_type>sensor_msgs/LaserScan</output_type>
          <frame_name>{name}_link</frame_name>
        </plugin>
      </sensor>
    </link>
  </model>
</sdf>'''

    @staticmethod
    def build_lidar_3d_sdf(name, namespace='',
                           horizontal_samples=1800, vertical_lines=16,
                           range_min=0.1, range_max=100.0,
                           x=0, y=0, z=0.15):
        """构建 3D LiDAR（Velodyne VLP-16 风格）的 SDF 字符串

        Args:
            name: 传感器名称
            namespace: ROS 命名空间
            horizontal_samples: 水平采样点数
            vertical_lines: 垂直激光线数（例如 16, 32, 64）
            range_min/max: 测距范围（米）
        """
        ns_tag = f'<namespace>{namespace}</namespace>' if namespace else ''
        return f'''<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="{name}">
    <pose>{x} {y} {z} 0 0 0</pose>
    <link name="{name}_link">
      <visual name="visual">
        <geometry><cylinder><radius>0.1</radius><length>0.07</length></cylinder></geometry>
        <material><ambient>0.8 0.8 0.8 1</ambient></material>
      </visual>
      <sensor name="lidar3d" type="ray">
        <always_on>1</always_on>
        <update_rate>10</update_rate>
        <ray>
          <scan>
            <horizontal>
              <samples>{horizontal_samples}</samples>
              <resolution>1</resolution>
              <min_angle>-3.14159</min_angle>
              <max_angle>3.14159</max_angle>
            </horizontal>
            <vertical>
              <samples>{vertical_lines}</samples>
              <resolution>1</resolution>
              <min_angle>-0.261799</min_angle>
              <max_angle>0.261799</max_angle>
            </vertical>
          </scan>
          <range>
            <min>{range_min}</min>
            <max>{range_max}</max>
            <resolution>0.02</resolution>
          </range>
        </ray>
        <plugin name="velodyne_plugin" filename="libgazebo_ros_ray_sensor.so">
          <ros>{ns_tag}</ros>
          <output_type>sensor_msgs/PointCloud2</output_type>
          <frame_name>{name}_link</frame_name>
        </plugin>
      </sensor>
    </link>
  </model>
</sdf>'''

    @staticmethod
    def build_camera_sdf(name, namespace='', width=640, height=480, fps=30,
                         x=0, y=0, z=0.15, roll=0, pitch=0, yaw=0):
        """构建 RGB Camera 的 SDF 字符串

        Args:
            name: 传感器名称
            namespace: ROS 命名空间
            width/height: 图像分辨率
            fps: 帧率
            roll, pitch, yaw: 姿态（弧度）
        """
        ns_tag = f'<namespace>{namespace}</namespace>' if namespace else ''
        return f'''<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="{name}">
    <pose>{x} {y} {z} {roll} {pitch} {yaw}</pose>
    <link name="{name}_link">
      <visual name="visual">
        <geometry><box><size>0.02 0.04 0.02</size></box></geometry>
        <material><ambient>0 1 0 1</ambient></material>
      </visual>
      <sensor name="camera" type="camera">
        <always_on>1</always_on>
        <update_rate>{fps}</update_rate>
        <visualize>true</visualize>
        <camera>
          <horizontal_fov>1.047</horizontal_fov>
          <image>
            <width>{width}</width>
            <height>{height}</height>
            <format>R8G8B8</format>
          </image>
          <clip>
            <near>0.1</near>
            <far>100</far>
          </clip>
          <noise>
            <type>gaussian</type>
            <mean>0.0</mean>
            <stddev>0.007</stddev>
          </noise>
        </camera>
        <plugin name="camera_plugin" filename="libgazebo_ros_camera.so">
          <ros>{ns_tag}</ros>
          <frame_name>{name}_link</frame_name>
        </plugin>
      </sensor>
    </link>
  </model>
</sdf>'''

    @staticmethod
    def build_depth_camera_sdf(name, namespace='', width=640, height=480, fps=30,
                               x=0, y=0, z=0.15):
        """构建深度相机的 SDF 字符串

        Args:
            name: 传感器名称
            namespace: ROS 命名空间
            width/height: 图像分辨率
            fps: 帧率
        """
        ns_tag = f'<namespace>{namespace}</namespace>' if namespace else ''
        return f'''<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="{name}">
    <pose>{x} {y} {z} 0 0 0</pose>
    <link name="{name}_link">
      <visual name="visual">
        <geometry><box><size>0.02 0.05 0.03</size></box></geometry>
        <material><ambient>0 0.6 1 1</ambient></material>
      </visual>
      <sensor name="depth_camera" type="depth_camera">
        <always_on>1</always_on>
        <update_rate>{fps}</update_rate>
        <camera>
          <horizontal_fov>1.047</horizontal_fov>
          <image>
            <width>{width}</width>
            <height>{height}</height>
          </image>
          <clip><near>0.1</near><far>30</far></clip>
        </camera>
        <plugin name="depth_plugin" filename="libgazebo_ros_depth_camera.so">
          <ros>{ns_tag}</ros>
          <frame_name>{name}_link</frame_name>
        </plugin>
      </sensor>
    </link>
  </model>
</sdf>'''

    @staticmethod
    def build_imu_sdf(name, namespace='', x=0, y=0, z=0.15):
        """构建 IMU 的 SDF 字符串

        Args:
            name: 传感器名称
            namespace: ROS 命名空间
        """
        ns_tag = f'<namespace>{namespace}</namespace>' if namespace else ''
        return f'''<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="{name}">
    <pose>{x} {y} {z} 0 0 0</pose>
    <link name="{name}_link">
      <sensor name="imu" type="imu">
        <always_on>1</always_on>
        <update_rate>200</update_rate>
        <imu>
          <angular_velocity>
            <x><noise type="gaussian"><mean>0</mean><stddev>0.00017</stddev></noise></x>
            <y><noise type="gaussian"><mean>0</mean><stddev>0.00017</stddev></noise></y>
            <z><noise type="gaussian"><mean>0</mean><stddev>0.00017</stddev></noise></z>
          </angular_velocity>
          <linear_acceleration>
            <x><noise type="gaussian"><mean>0</mean><stddev>0.001</stddev></noise></x>
            <y><noise type="gaussian"><mean>0</mean><stddev>0.001</stddev></noise></y>
            <z><noise type="gaussian"><mean>0</mean><stddev>0.001</stddev></noise></z>
          </linear_acceleration>
        </imu>
        <plugin name="imu_plugin" filename="libgazebo_ros_imu_sensor.so">
          <ros>{ns_tag}</ros>
          <frame_name>{name}_link</frame_name>
        </plugin>
      </sensor>
    </link>
  </model>
</sdf>'''

    # ============================
    #  公共接口
    # ============================

    def add_sensor(self, name, sdf_xml, namespace=''):
        """通用传感器添加接口

        Args:
            name: 传感器名称（仿真中唯一标识）
            sdf_xml: SDF 格式的传感器描述字符串
            namespace: 传感器命名空间（为空时使用 name）
        """
        req = SpawnEntity.Request()
        req.name = name
        req.xml = sdf_xml
        req.robot_namespace = namespace if namespace else name
        future = self.spawn_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if future.result() and future.result().success:
            self.get_logger().info(f'[OK] 传感器已添加: {name}')
            return True
        msg = future.result().status_message if future.result() else '无响应'
        self.get_logger().error(f'[FAIL] 添加失败: {msg}')
        return False

    def add_lidar_2d(self, name, **kwargs):
        ns = kwargs.pop('namespace', '')
        return self.add_sensor(name, self.build_lidar_2d_sdf(name, namespace=ns, **kwargs), ns)

    def add_lidar_3d(self, name, **kwargs):
        ns = kwargs.pop('namespace', '')
        return self.add_sensor(name, self.build_lidar_3d_sdf(name, namespace=ns, **kwargs), ns)

    def add_camera(self, name, **kwargs):
        ns = kwargs.pop('namespace', '')
        return self.add_sensor(name, self.build_camera_sdf(name, namespace=ns, **kwargs), ns)

    def add_depth_camera(self, name, **kwargs):
        ns = kwargs.pop('namespace', '')
        return self.add_sensor(name, self.build_depth_camera_sdf(name, namespace=ns, **kwargs), ns)

    def add_imu(self, name, **kwargs):
        ns = kwargs.pop('namespace', '')
        return self.add_sensor(name, self.build_imu_sdf(name, namespace=ns, **kwargs), ns)

    def remove_sensor(self, name):
        """删除传感器"""
        req = DeleteEntity.Request()
        req.name = name
        future = self.delete_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.result() and future.result().success:
            self.get_logger().info(f'[OK] 传感器已删除: {name}')
            return True
        msg = future.result().status_message if future.result() else '无响应'
        self.get_logger().error(f'[FAIL] 删除失败: {msg}')
        return False

    def list_models(self):
        """列出当前仿真中的所有模型"""
        req = GetModelList.Request()
        future = self.model_list_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        return future.result().model_names if future.result() else []


def interactive_mode(mgr):
    """交互式命令行界面"""
    print("=" * 50)
    print("     传感器动态增删管理工具")
    print("     2D LiDAR | 3D LiDAR | Camera | Depth | IMU")
    print("=" * 50)

    while rclpy.ok():
        print("\n操作选项:")
        print("  1. 添加 2D LiDAR")
        print("  2. 添加 3D LiDAR (Velodyne VLP-16)")
        print("  3. 添加 RGB Camera")
        print("  4. 添加 Depth Camera")
        print("  5. 添加 IMU")
        print("  6. 删除传感器")
        print("  7. 列出所有模型")
        print("  0. 退出")
        choice = input("请选择: ").strip()

        if choice == '1':
            name = input("名称: ").strip()
            ns = input("命名空间 (回车跳过): ").strip()
            x = input("X 坐标 (默认 0): ").strip() or "0"
            y = input("Y 坐标 (默认 0): ").strip() or "0"
            z = input("Z 坐标 (默认 0.15): ").strip() or "0.15"
            mgr.add_lidar_2d(name, namespace=ns, x=float(x), y=float(y), z=float(z))

        elif choice == '2':
            name = input("名称: ").strip()
            ns = input("命名空间 (回车跳过): ").strip()
            mgr.add_lidar_3d(name, namespace=ns)

        elif choice == '3':
            name = input("名称: ").strip()
            ns = input("命名空间 (回车跳过): ").strip()
            x = input("X 坐标 (默认 0): ").strip() or "0"
            y = input("Y 坐标 (默认 0): ").strip() or "0"
            z = input("Z 坐标 (默认 0.15): ").strip() or "0.15"
            mgr.add_camera(name, namespace=ns, x=float(x), y=float(y), z=float(z))

        elif choice == '4':
            name = input("名称: ").strip()
            ns = input("命名空间 (回车跳过): ").strip()
            mgr.add_depth_camera(name, namespace=ns)

        elif choice == '5':
            name = input("名称: ").strip()
            ns = input("命名空间 (回车跳过): ").strip()
            mgr.add_imu(name, namespace=ns)

        elif choice == '6':
            name = input("传感器名称: ").strip()
            mgr.remove_sensor(name)

        elif choice == '7':
            models = mgr.list_models()
            print(f"\n当前仿真中模型 ({len(models)} 个):")
            for m in sorted(models):
                print(f"  - {m}")

        elif choice == '0':
            break


def demo_mode(mgr):
    """演示模式：依次添加/删除各类传感器"""
    mgr.get_logger().info('=== 演示模式开始 ===')

    mgr.add_lidar_2d('demo_lidar_2d', namespace='demo', x=1.0, y=0)
    mgr.add_lidar_3d('demo_lidar_3d', namespace='demo', x=-1.0, y=0)
    mgr.add_camera('demo_camera', namespace='demo', x=2.0, y=0)
    mgr.add_imu('demo_imu', namespace='demo', x=0, y=1.0)

    rclpy.spin_once(mgr, timeout_sec=2.0)

    models = mgr.list_models()
    mgr.get_logger().info(f'当前模型: {models}')

    mgr.remove_sensor('demo_lidar_2d')
    mgr.get_logger().info('=== 演示模式结束 ===')


def main():
    rclpy.init(args=sys.argv)
    mgr = SensorManager()

    if len(sys.argv) > 1 and sys.argv[1] == '--demo':
        demo_mode(mgr)
    else:
        interactive_mode(mgr)

    mgr.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
