#!/usr/bin/env python3
"""
预定义演示场景
一键执行常见仿真操作组合

场景1: 教室导航    — 生成机器人到教室，开始导航
场景2: 多机器人协作 — 生成3台机器人到不同位置
场景3: 传感器套件  — 为机器人添加全套传感器测试
场景4: 场景清理    — 删除所有动态实体

用法:
  python3 demo_scenarios.py              # 查看场景列表
  python3 demo_scenarios.py 1            # 执行场景 1
  python3 demo_scenarios.py 2            # 执行场景 2
  python3 demo_scenarios.py 3            # 执行场景 3
  python3 demo_scenarios.py 4            # 执行场景 4
"""

import rclpy
from rclpy.node import Node
import sys
import os
import time


class ScenarioRunner(Node):
    def __init__(self):
        super().__init__('scenario_runner')
        sys.path.insert(0, os.path.dirname(__file__))
        from spawn_robot import RobotSpawner
        from manage_sensors import SensorManager

        self.spawner = RobotSpawner()
        self.sensor_mgr = SensorManager()

        self.sdf_path = os.path.abspath(os.path.join(
            os.path.dirname(__file__),
            '..', 'models', 'custom_robot', 'model.sdf'
        ))

    def scenario_1_classroom_navigation(self):
        """场景1: 教室导航 — 单机器人 + LiDAR"""
        self.get_logger().info('=' * 50)
        self.get_logger().info('场景1: 教室导航仿真')
        self.get_logger().info('=' * 50)

        self.spawner.spawn_robot_from_file(
            'nav_robot', self.sdf_path,
            x=-2.0, y=0.0, z=0.1
        )
        time.sleep(1)
        self.get_logger().info('机器人已就位在教室后部，可用于 SLAM 导航测试')
        self.get_logger().info('发布话题: /nav_robot/cmd_vel, /nav_robot/scan, /nav_robot/odom')

    def scenario_2_multi_robot(self):
        """场景2: 多机器人协作"""
        self.get_logger().info('=' * 50)
        self.get_logger().info('场景2: 多机器人编队')
        self.get_logger().info('=' * 50)

        formations = [
            ('robot_leader', -2.0, 0.0),
            ('robot_left', -3.0, -1.0),
            ('robot_right', -3.0, 1.0),
        ]
        for name, x, y in formations:
            self.spawner.spawn_robot_from_file(name, self.sdf_path,
                                               x=x, y=y, z=0.1)
            time.sleep(0.5)

        self.get_logger().info('3台机器人已编队部署完成')
        self.get_logger().info('控制 leader: ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/robot_leader/cmd_vel')

    def scenario_3_sensor_suite(self):
        """场景3: 传感器套件测试"""
        self.get_logger().info('=' * 50)
        self.get_logger().info('场景3: 全套传感器测试')
        self.get_logger().info('=' * 50)

        self.spawner.spawn_robot_from_file(
            'sensor_bot', self.sdf_path,
            x=-1.0, y=0.0, z=0.1
        )
        time.sleep(1)

        self.sensor_mgr.add_lidar_2d('extra_lidar', namespace='extra',
                                     x=-1.0, y=0, z=0.25)
        self.sensor_mgr.add_lidar_3d('extra_3d_lidar', namespace='extra',
                                     x=-1.0, y=0.5, z=0.25)
        self.sensor_mgr.add_camera('extra_camera', namespace='extra',
                                   x=-1.0, y=-0.5, z=0.25)
        self.sensor_mgr.add_depth_camera('extra_depth', namespace='extra',
                                         x=-1.0, y=0.8, z=0.25)
        time.sleep(1)

        self.get_logger().info('传感器套件已部署完成，可通过 ros2 topic list 查看')
        self.get_logger().info('话题包括: /extra/scan, /extra/pointcloud, /extra/image_raw 等')

    def scenario_4_cleanup(self):
        """场景4: 清理"""
        self.get_logger().info('=' * 50)
        self.get_logger().info('场景4: 清理所有动态实体')
        self.get_logger().info('=' * 50)

        models = self.spawner.list_models()
        static_models = {'ground_plane', 'floor', 'platform',
                         'wall_north', 'wall_south', 'wall_east_lower',
                         'wall_east_above_door', 'wall_east_upper', 'wall_west',
                         'window_1', 'window_1_frame', 'window_2', 'window_2_frame',
                         'window_3', 'window_3_frame',
                         'door_frame', 'door_panel',
                         'blackboard', 'blackboard_frame',
                         'wall_clock', 'wall_clock_hand_hour', 'wall_clock_hand_minute',
                         'podium', 'podium_legs', 'trash_bin'}
        for i in [2, 3, 4]:
            for j in range(1, 7):
                static_models.add(f'desk_{i}_{j}')
                static_models.add(f'chair_{i}_{j}')

        dynamic_models = [m for m in models if m not in static_models]
        for model in dynamic_models:
            self.spawner.delete_robot(model)
            time.sleep(0.2)

        self.get_logger().info(f'已清理 {len(dynamic_models)} 个动态实体')


def main():
    rclpy.init(args=sys.argv)
    runner = ScenarioRunner()

    scenarios = {
        '1': ('教室导航', runner.scenario_1_classroom_navigation),
        '2': ('多机器人编队', runner.scenario_2_multi_robot),
        '3': ('传感器套件测试', runner.scenario_3_sensor_suite),
        '4': ('清理场景', runner.scenario_4_cleanup),
    }

    if len(sys.argv) > 1 and sys.argv[1] in scenarios:
        scenarios[sys.argv[1]][1]()
    else:
        print("=" * 50)
        print("     预定义仿真场景")
        print("=" * 50)
        for key, (desc, _) in scenarios.items():
            print(f"  {key}. {desc}")
        print("\n用法: python3 demo_scenarios.py <场景编号>")
        print("示例: python3 demo_scenarios.py 1")

    runner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
