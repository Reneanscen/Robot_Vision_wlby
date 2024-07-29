import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():
    # 获取参数文件路径
    param_file = os.path.join(
        get_package_share_directory('zme_fusion_map'),
        'config',
        'config.yaml'
    )

    print('fusion_map will be load param.yaml path:')
    print(param_file)

    ##use_gdb = LaunchConfiguration('use_gdb', default='false')

    # 启动 C++ 节点
    my_cpp_node = Node(
        package='zme_fusion_map',
        executable='zme_fusion_map_node',
        output='screen',
        parameters=[param_file],
        #prefix='gdb -ex run --args'
    )


    return LaunchDescription([my_cpp_node])
