import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


# 定义函数名称为：generate_launch_description
def generate_launch_description():

    param_file2 = os.path.join(
        get_package_share_directory('yolov8_ncnn'),
        'config',
        'daibai_dcl.yaml'
    )

    print('mask rcnn node will be load param.yaml path:')
    print(param_file2)

    yolov8_node = Node(
        package="yolov8_ncnn",
        executable="yolov8_ncnn_node",
        output = "both",
        parameters=[param_file2]
    )
    # 创建LaunchDescription对象launch_description,用于描述launch文件
    launch_description = LaunchDescription([yolov8_node])
    # 返回让ROS2根据launch描述执行节点
    return launch_description
