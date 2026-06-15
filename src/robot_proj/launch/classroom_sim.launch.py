"""
教室仿真环境启动文件
一键启动 Gazebo + 教室场景 + 初始机器人

用法:
  ros2 launch robot_proj classroom_sim.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory, get_packages_with_prefixes
from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo


def _clean_gazebo_model_path():
    """构建干净的 GAZEBO_MODEL_PATH，过滤掉不含 model.config 的目录。"""
    from catkin_pkg.package import parse_package, InvalidPackage
    PACKAGE_MANIFEST_FILENAME = 'package.xml'

    path_parts = []
    for pkg_name in get_packages_with_prefixes():
        pkg_share = get_package_share_directory(pkg_name)
        pkg_xml = os.path.join(pkg_share, PACKAGE_MANIFEST_FILENAME)
        if not os.path.isfile(pkg_xml):
            continue
        try:
            pkg = parse_package(pkg_xml)
        except InvalidPackage:
            continue
        for export in pkg.exports:
            if export.tagname != 'gazebo_ros':
                continue
            raw = export.attributes.get('gazebo_model_path', '')
            if not raw:
                continue
            raw = raw.replace('${prefix}', pkg_share)
            for part in raw.split(':'):
                part = os.path.normpath(part)
                if os.path.isdir(part):
                    path_parts.append(part)

    extra = os.environ.get('GAZEBO_MODEL_PATH', '')
    if extra:
        for part in extra.split(':'):
            part = os.path.normpath(part)
            if part and os.path.isdir(part) and part not in path_parts:
                path_parts.append(part)

    return ':'.join(filter(None, path_parts))


def generate_launch_description():
    pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    world_file = os.path.join(pkg_dir, 'worlds', 'classroom.world')
    robot_sdf = os.path.join(pkg_dir, 'models', 'custom_robot', 'model.sdf')

    gazebo_model_path = _clean_gazebo_model_path()

    gzserver = ExecuteProcess(
        cmd=[
            'gzserver', world_file,
            '--verbose',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
            '-s', 'libgazebo_ros_force_system.so',
        ],
        output='screen',
        additional_env={
            'GAZEBO_MODEL_PATH': gazebo_model_path,
        },
    )

    gzclient = ExecuteProcess(
        cmd=[
            'gzclient',
            '--gui-client-plugin=libgazebo_ros_eol_gui.so',
        ],
        output='screen',
        additional_env={
            'GAZEBO_MODEL_PATH': gazebo_model_path,
        },
    )

    spawn_robot = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
            '-entity', 'robot_01',
            '-file', robot_sdf,
            '-robot_namespace', 'robot_01',
            '-x', '-2.0',
            '-y', '0.0',
            '-z', '0.1',
        ],
        output='screen',
        additional_env={'GAZEBO_MODEL_PATH': gazebo_model_path},
    )

    return LaunchDescription([
        gzserver,
        gzclient,
        spawn_robot,
        LogInfo(msg=['===== 教室仿真环境启动完成 =====']),
        LogInfo(msg=['查看 Topics: ros2 topic list']),
        LogInfo(msg=['键盘控制: ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/robot_01/cmd_vel']),
    ])
