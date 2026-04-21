"""
tof_bringup — bridge 노드 + static TF + (옵션) RViz2.

사용 예:
  ros2 launch tof_bringup bringup.launch.py
  ros2 launch tof_bringup bringup.launch.py rviz:=true
  ros2 launch tof_bringup bringup.launch.py can_interface:=can1
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory('tof_bringup')

    params_file = LaunchConfiguration('params_file')
    can_interface = LaunchConfiguration('can_interface')
    rviz = LaunchConfiguration('rviz')
    rviz_config = LaunchConfiguration('rviz_config')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_share, 'config', 'bridge_params.yaml'),
            description='bridge_node 파라미터 YAML 경로',
        ),
        DeclareLaunchArgument(
            'can_interface',
            default_value='can0',
            description='SocketCAN 인터페이스 (YAML 값을 덮어씀)',
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            description='true 로 지정하면 RViz2 동시 실행',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(pkg_share, 'rviz', 'tof.rviz'),
            description='RViz2 설정 파일 경로',
        ),

        # --- bridge 노드 ---
        Node(
            package='tof_can_bridge',
            executable='bridge_node',
            name='tof_can_bridge',
            output='screen',
            parameters=[
                params_file,
                {'can_interface': can_interface},
            ],
        ),

        # --- static TF (임시 원점) ---
        # 센서 장착 위치 확정되면 translation/rotation 을 실제 값으로 교체.
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='tf_l4cd',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'l4cd_frame'],
        ),
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='tf_l5cx',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'l5cx_optical_frame'],
        ),
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='tf_l7cx',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'l7cx_optical_frame'],
        ),

        # --- RViz2 (옵션) ---
        Node(
            package='rviz2', executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            condition=IfCondition(rviz),
        ),
    ])
