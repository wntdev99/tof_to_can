"""
tof_eval — 단일 PointCloud2 토픽에 대해 평가 노드 실행.

사용 예:
  # eval_params.yaml 파일 직접 지정
  ros2 launch tof_eval eval.launch.py \\
      params_file:=/absolute/path/to/eval_params.yaml

  # input_topic / session_id / csv_output_dir 은 launch arg 로 YAML 값 override 가능
  ros2 launch tof_eval eval.launch.py \\
      params_file:=/path/to/params.yaml \\
      input_topic:=/tof/l7cx/points \\
      session_id:=l7cx_matte_indoor_001
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _make_node(context, *args, **kwargs):
    params_file = LaunchConfiguration('params_file').perform(context)
    input_topic = LaunchConfiguration('input_topic').perform(context)
    session_id = LaunchConfiguration('session_id').perform(context)
    csv_output_dir = LaunchConfiguration('csv_output_dir').perform(context)

    overrides = {}
    if input_topic:
        overrides['input_topic'] = input_topic
    if session_id:
        overrides['scene.session_id'] = session_id
    if csv_output_dir:
        overrides['csv_output_dir'] = csv_output_dir

    params = [params_file]
    if overrides:
        params.append(overrides)

    return [Node(
        package='tof_eval',
        executable='eval_node',
        name='tof_eval',
        output='screen',
        parameters=params,
    )]


def generate_launch_description():
    pkg_share = get_package_share_directory('tof_eval')
    default_params = os.path.join(pkg_share, 'config', 'eval_params.example.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='eval_params YAML 경로',
        ),
        DeclareLaunchArgument(
            'input_topic',
            default_value='',
            description='YAML 의 input_topic 을 덮어쓸 값 (빈 문자열이면 YAML 값 그대로)',
        ),
        DeclareLaunchArgument(
            'session_id',
            default_value='',
            description='YAML 의 scene.session_id 를 덮어쓸 값',
        ),
        DeclareLaunchArgument(
            'csv_output_dir',
            default_value='',
            description='YAML 의 csv_output_dir 을 덮어쓸 값',
        ),
        OpaqueFunction(function=_make_node),
    ])
