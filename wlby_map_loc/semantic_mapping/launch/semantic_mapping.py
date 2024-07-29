import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取参数文件路径
    param_file = os.path.join(
        get_package_share_directory('semantic_mapping'),
        'config',
        'daibai_dcl.yaml'
    )

    print('semantic_mapping_node will be load param.yaml path:')
    print(param_file)

    param_file2 = os.path.join(
        get_package_share_directory('yolov8_ncnn'),
        'config',
        'daibai_dcl.yaml'
    )

    print('yolov8_ncnn_node will be load param.yaml path:')
    print(param_file2)
    

    # 启动 C++ 节点
    my_cpp_node = Node(
        package='semantic_mapping',
        executable='semantic_mapping_node',
        output='screen',
        parameters=[param_file]
    )
    mask_rcnn_node = Node(
        package="yolov8_ncnn",
        executable="yolov8_ncnn_node",
        output = "both",
        parameters=[param_file2]
    )
    
    #mask_rcnn_node = Node(
    #    package="mask_rcnn_ros_pytorch",
    #    executable="mask_rcnn_node",
    #    output = "both",
    #    parameters=[param_file2]
    #)

    return LaunchDescription([my_cpp_node,mask_rcnn_node])
