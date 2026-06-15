#!/usr/bin/env python3
"""
动态机器人增删工具
使用 Gazebo ROS2 /spawn_entity 和 /delete_entity 服务实现

功能:
  - 从 SDF 字符串动态生成机器人到仿真中
  - 从 SDF 文件加载并生成机器人
  - 按名称删除机器人
  - 批量删除指定前缀的机器人
  - 列出当前仿真中所有模型
  - 交互式命令行界面
  - --demo 演示模式

用法:
  python3 spawn_robot.py              # 交互模式
  python3 spawn_robot.py --demo       # 演示模式
"""

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity, DeleteEntity, GetModelList
import sys
import os


class RobotSpawner(Node):
    """动态管理仿真中的机器人模型"""

    def __init__(self):
        super().__init__('robot_spawner')

        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.delete_client = self.create_client(DeleteEntity, '/delete_entity')
        self.model_list_client = self.create_client(GetModelList, '/get_model_list')

        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待 /spawn_entity 服务...')

        while not self.delete_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待 /delete_entity 服务...')

        while not self.model_list_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待 /get_model_list 服务...')

        self.get_logger().info('RobotSpawner 就绪')

    def spawn_robot(self, name, model_xml, x=0.0, y=0.0, z=0.0,
                    roll=0.0, pitch=0.0, yaw=0.0):
        """生成机器人到仿真中

        Args:
            name: 机器人名称（在仿真中唯一标识）
            model_xml: SDF 格式的模型描述字符串
            x, y, z: 初始位置（米）
            roll, pitch, yaw: 初始姿态（弧度）
        """
        req = SpawnEntity.Request()
        req.name = name
        req.xml = model_xml
        req.robot_namespace = name
        req.initial_pose.position.x = float(x)
        req.initial_pose.position.y = float(y)
        req.initial_pose.position.z = float(z)
        req.initial_pose.orientation.x = float(roll)
        req.initial_pose.orientation.y = float(pitch)
        req.initial_pose.orientation.z = float(yaw)
        req.initial_pose.orientation.w = 1.0
        req.reference_frame = 'world'

        future = self.spawn_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if future.result() is not None and future.result().success:
            self.get_logger().info(f'[OK] 已生成机器人: {name} at ({x}, {y}, {z})')
            return True
        else:
            msg = future.result().status_message if future.result() else '无响应'
            self.get_logger().error(f'[FAIL] 生成失败: {msg}')
            return False

    def spawn_robot_from_file(self, name, sdf_file_path, x=0.0, y=0.0, z=0.0,
                              roll=0.0, pitch=0.0, yaw=0.0):
        """从 SDF 文件加载并生成机器人"""
        if not os.path.exists(sdf_file_path):
            self.get_logger().error(f'文件不存在: {sdf_file_path}')
            return False
        with open(sdf_file_path, 'r') as f:
            model_xml = f.read()
        return self.spawn_robot(name, model_xml, x, y, z, roll, pitch, yaw)

    def delete_robot(self, name):
        """从仿真中删除机器人"""
        req = DeleteEntity.Request()
        req.name = name

        future = self.delete_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None and future.result().success:
            self.get_logger().info(f'[OK] 已删除: {name}')
            return True
        else:
            msg = future.result().status_message if future.result() else '无响应'
            self.get_logger().error(f'[FAIL] 删除失败: {msg}')
            return False

    def list_models(self):
        """列出当前仿真中的所有模型"""
        req = GetModelList.Request()
        future = self.model_list_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.result():
            return future.result().model_names
        return []

    def delete_all_robots(self, prefix='robot_'):
        """批量删除以指定前缀开头的机器人"""
        models = self.list_models()
        deleted = 0
        for model in models:
            if model.startswith(prefix):
                if self.delete_robot(model):
                    deleted += 1
        self.get_logger().info(f'共删除 {deleted} 个机器人')


def interactive_mode(spawner):
    """交互式命令行界面"""
    sdf_path = os.path.join(
        os.path.dirname(__file__),
        '..', 'models', 'custom_robot', 'model.sdf'
    )
    sdf_path = os.path.abspath(sdf_path)

    print("=" * 50)
    print("     机器人动态增删管理工具")
    print("=" * 50)

    while rclpy.ok():
        print("\n操作选项:")
        print("  1. 生成一台机器人")
        print("  2. 批量生成机器人")
        print("  3. 删除指定机器人")
        print("  4. 列出所有模型")
        print("  5. 清空所有机器人")
        print("  6. 从自定义 SDF 生成")
        print("  0. 退出")
        choice = input("请选择: ").strip()

        if choice == '1':
            name = input("机器人名称 (默认 robot_01): ").strip() or "robot_01"
            x = input("X 坐标 (默认 0): ").strip() or "0"
            y = input("Y 坐标 (默认 0): ").strip() or "0"
            z = input("Z 坐标 (默认 0.1): ").strip() or "0.1"
            spawner.spawn_robot_from_file(name, sdf_path, float(x), float(y), float(z))

        elif choice == '2':
            count = int(input("生成数量: ").strip() or "3")
            base_x = float(input("起始 X (默认 0): ").strip() or "0")
            base_y = float(input("起始 Y (默认 0): ").strip() or "0")
            spacing = float(input("间距 (默认 1.5): ").strip() or "1.5")
            for i in range(count):
                name = f"robot_{i+1:02d}"
                x = base_x + i * spacing
                spawner.spawn_robot_from_file(name, sdf_path, x, base_y)

        elif choice == '3':
            name = input("要删除的机器人名称: ").strip()
            spawner.delete_robot(name)

        elif choice == '4':
            models = spawner.list_models()
            print(f"\n当前仿真中模型 ({len(models)} 个):")
            for m in sorted(models):
                print(f"  - {m}")

        elif choice == '5':
            prefix = input("机器人名称前缀 (默认 robot_): ").strip() or "robot_"
            spawner.delete_all_robots(prefix)

        elif choice == '6':
            name = input("机器人名称: ").strip()
            custom_sdf = input("SDF 文件路径: ").strip()
            x = float(input("X 坐标: ").strip() or "0")
            y = float(input("Y 坐标: ").strip() or "0")
            z = float(input("Z 坐标: ").strip() or "0.1")
            spawner.spawn_robot_from_file(name, custom_sdf, x, y, z)

        elif choice == '0':
            break


def main():
    rclpy.init(args=sys.argv)
    spawner = RobotSpawner()

    if len(sys.argv) > 1 and sys.argv[1] == '--demo':
        sdf_path = os.path.join(
            os.path.dirname(__file__),
            '..', 'models', 'custom_robot', 'model.sdf'
        )
        sdf_path = os.path.abspath(sdf_path)
        spawner.get_logger().info('演示模式: 批量生成 3 台机器人')
        for i in range(3):
            name = f"robot_{i+1:02d}"
            spawner.spawn_robot_from_file(name, sdf_path, i * 1.5, 0.0, 0.1)
        rclpy.spin_once(spawner, timeout_sec=1.0)
        spawner.get_logger().info('演示: 删除 robot_02')
        spawner.delete_robot('robot_02')
    else:
        interactive_mode(spawner)

    spawner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
