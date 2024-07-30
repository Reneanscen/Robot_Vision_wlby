import os.path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

from launch_ros.actions import Node

def generate_launch_description():
    # 配置常量
    package_path = get_package_share_directory('zme_birdvision')
    default_config_path = os.path.join(package_path, 'config', '500_pinhole_bird_vision.yaml')
    default_rviz_config_path = os.path.join(package_path, 'config', 'bird_vision.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    config_path = LaunchConfiguration('config_path')
    rviz_use = LaunchConfiguration('rviz')
    rviz_cfg = LaunchConfiguration('rviz_cfg')

    # 参数声明：声明一个参数的名称、类型和默认值，在启动过程中可以使用该参数来配置节点或其他组件的行为
    declare_use_sim_time_cmd = DeclareLaunchArgument('use_sim_time',
                                                     default_value='false',
                                                     description='Use simulation (Gazebo) clock if true'
                                                     )

    declare_config_path_cmd = DeclareLaunchArgument('config_path',
                                                    default_value=default_config_path,
                                                    description='Yaml config file path'
                                                    )

    declare_rviz_cmd = DeclareLaunchArgument('rviz',
                                             default_value='false',
                                             description='Use RViz to monitor results'
                                             )

    declare_rviz_config_path_cmd = DeclareLaunchArgument('rviz_cfg',
                                                         default_value=default_rviz_config_path,
                                                         description='RViz config file path'
                                                         )

    ## 定位建图节点
    zme_birdvision_node = Node(
        package='zme_birdvision',
        executable='zme_birdvision_node',
        parameters=[config_path,
                    {'use_sim_time': use_sim_time}],
        output='screen'
    )

    ## Rviz2节点
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg],
        condition=IfCondition(rviz_use)
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_config_path_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_rviz_config_path_cmd)

    ld.add_action(zme_birdvision_node)
    ld.add_action(rviz_node)

    return ld