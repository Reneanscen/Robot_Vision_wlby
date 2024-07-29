import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取参数文件路径
    param_file = os.path.join(
        get_package_share_directory('zme_charging_indentifi'),
        'config',
        'config.yaml'
    )

    print('zme_charging_indentifi will be load param.yaml path:')
    print(param_file)

   

    # 启动 C++ 节点
    my_cpp_node = Node(
        package='zme_charging_indentifi',
        executable='zme_charging_indentifi_node',
        output='screen',
        #prefix=['gdb -ex run --args'],
        parameters=[param_file]
    )


    return LaunchDescription([my_cpp_node])
