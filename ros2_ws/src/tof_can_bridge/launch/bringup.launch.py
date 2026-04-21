"""
사용 예:
  ros2 launch tof_can_bridge bringup.launch.py can_interface:=can0

TF:
  각 센서 frame 을 `base_link` 에 대해 원하는 위치로 static TF publisher
  로 연결해야 rviz2 의 PointCloud2 / Range 표시가 올바르게 나온다.
  여기서는 기본값으로 각 frame 을 원점에 공용으로 붙여 둔다.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    can_interface = LaunchConfiguration('can_interface')
    frame_l4cd = LaunchConfiguration('frame_id_l4cd')
    frame_l5cx = LaunchConfiguration('frame_id_l5cx')
    frame_l7cx = LaunchConfiguration('frame_id_l7cx')

    return LaunchDescription([
        DeclareLaunchArgument('can_interface',  default_value='can0'),
        DeclareLaunchArgument('frame_id_l4cd',  default_value='l4cd_frame'),
        DeclareLaunchArgument('frame_id_l5cx',  default_value='l5cx_optical_frame'),
        DeclareLaunchArgument('frame_id_l7cx',  default_value='l7cx_optical_frame'),

        Node(
            package='tof_can_bridge',
            executable='bridge_node',
            name='tof_can_bridge',
            output='screen',
            parameters=[{
                'can_interface': can_interface,
                'frame_id_l4cd': frame_l4cd,
                'frame_id_l5cx': frame_l5cx,
                'frame_id_l7cx': frame_l7cx,
            }],
        ),

        # --- static TF (임시) ---
        # 센서 장착 위치가 확정되면 실제 translation/rotation 으로 수정.
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', frame_l4cd],
            name='tf_l4cd',
        ),
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', frame_l5cx],
            name='tf_l5cx',
        ),
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', frame_l7cx],
            name='tf_l7cx',
        ),
    ])
