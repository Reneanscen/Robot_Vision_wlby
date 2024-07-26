import os

from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition,UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():
    bundle_cfg_1 = os.path.join(
        get_package_share_directory('stag_ros'),
        'cfg',
        'bundle_4.yaml'
        )

    bundle_cfg_2 = os.path.join(
        get_package_share_directory('stag_ros'),
        'cfg',
        'bundle_config_4.yaml'
        )    
    rviz_file = os.path.join(get_package_share_directory('stag_ros'), 'cfg',
                            'display.rviz')

    stag_node = Node(package='stag_ros',
        executable='stag_node',
        name='stag_node',
        # output='screen',
        parameters = [bundle_cfg_1,bundle_cfg_2],
        )

    rviz_node = Node(package='rviz2',
        executable='rviz2',
        name='stag_display',
        arguments=['--display-config', rviz_file],
        output='screen',
        )

    return LaunchDescription([
        stag_node,
        rviz_node
    ])


