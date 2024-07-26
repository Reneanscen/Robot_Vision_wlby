import os
import torch
import cv2
import numpy as np
import open3d as o3d
import pyrealsense2 as rs
import math

# import socket
# import VisionMsg0603_pb2
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Transform
from std_msgs.msg import Header
from vision_msgs.msg import DetectedObject, DetectedObjectList
from datetime import datetime

# import rospy
# from std_msgs.msg import String
# from VisionMsg_pb2 import VisionData

# camera_to_base = np.array([[-0.49684728612575363, 0.4158536238210313, 0.761714210071198, 0.1332792548908207],
#                            [0.2523896764262379, 0.9090165337061828, -0.3316449798839946, 0.5567822129530278],
#                            [-0.8303265776204758, 0.027471894796544105, -0.556599559372225, 0.8182465163041617],
#                            [0.0, 0.0, 0.0, 1.0]])

camera_to_base = np.array([[0.4830191777293635, -0.5454382934019704, 0.6849741177857716, 0.09830437700902608],
                           [-0.3508700489363267, -0.8372981068953109, -0.4193114462412742, 0.5652720778206468],
                           [0.8022360517360645, -0.03780143225164029, -0.5958090038046218, 0.794058946750924],
                           [0.0, 0.0, 0.0, 1.0]])

objectname = {
    0:"push pedal",
    1:"cup",
    2:"doorknob"
}


def object_2D_detection(img):
    # Model
    model = torch.hub.load(r'/home/lz/yolov5', 'custom', path=r'runs/train/best_20230620sep.pt', source='local', force_reload=True)

    # Inference
    results = model(img, size=1280)

    aa = results.pandas()
    print(aa)

    boxes = results.xyxy[0].tolist()

    torch.cuda.empty_cache()

    return boxes

def depth_to_pointcloud(depth_image, intrinsic):
    # Create Open3D Image from depth map
    o3d_depth = o3d.geometry.Image(depth_image)

    # Get intrinsic parameters
    fx, fy, cx, cy = intrinsic.fx, intrinsic.fy, intrinsic.ppx, intrinsic.ppy

    # Create Open3D PinholeCameraIntrinsic object
    o3d_intrinsic = o3d.camera.PinholeCameraIntrinsic(width=depth_image.shape[1], height=depth_image.shape[0], fx=fx, fy=fy, cx=cx, cy=cy)

    # Create Open3D PointCloud object from depth image and intrinsic parameters
    pcd = o3d.geometry.PointCloud.create_from_depth_image(o3d_depth, o3d_intrinsic)

    return pcd

def achieve_targetpointcloud(mask, depth, intrinsic):
    mask_resized = cv2.resize(mask, (depth.shape[1], depth.shape[0]))
    roi = cv2.bitwise_and(depth, depth, mask=mask_resized)
    target_pointcloud = depth_to_pointcloud(roi, intrinsic)
    return target_pointcloud

def generate_mask(img_shape, x1, y1, x2, y2):
    mask = np.zeros(img_shape[:2], dtype=np.uint8)
    mask[y1:y2, x1:x2] = 1
    return mask

def transform_matrix(x, y, z):
    # 生成平移矩阵
    translate = np.eye(4)
    translate[:-1, -1] = [x, y, z]

    # 生成旋转矩阵（单位矩阵）
    rotation = np.eye(4)

    # 将平移矩阵和旋转矩阵相乘得到变换矩阵
    transform = np.matmul(translate, rotation)

    return transform

def shrink_rect(x1, y1, x2, y2):
    # 计算原始矩形框的宽度和高度
    width = x2 - x1
    height = y2 - y1

    # 计算缩小 10% 后的宽度和高度
    new_width = math.floor(width * 0.9)
    new_height = math.floor(height * 0.9)

    # 计算缩小后矩形框的左上角坐标和右下角坐标
    new_x1 = math.ceil(x1 + (width - new_width) / 2)
    new_y1 = math.ceil(y1 + (height - new_height) / 2)
    new_x2 = math.floor(x2 - (width - new_width) / 2)
    new_y2 = math.floor(y2 - (height - new_height) / 2)

    return new_x1, new_y1, new_x2, new_y2


num_image = 0

# 初始化 RealSense 相机
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 15)
config.enable_stream(rs.stream.color, 1280, 720, rs.format.rgb8, 15)
profile = pipeline.start(config)

align_to = rs.stream.color
align = rs.align(align_to)

# 创建窗口并获取相机内参
# cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)


# rospy.init_node('xyz_publisher', anonymous=True)
# pub = rospy.Publisher('xyz_topic', String, queue_size=10)

# 创建XYZ消息对象
# xyz_msg = VisionData()

# # 创建一个 socket 对象
# server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#
# # 绑定 IP 地址和端口号
# server_address = ('192.168.124.248', 8888)
# server_socket.bind(server_address)
#
# # 监听连接
# server_socket.listen(1)

# # 等待客户端连接
# print("等待客户端连接...")
# client_socket, client_address = server_socket.accept()
# print("客户端已连接：", client_address)

rclpy.init()
node = Node("vision_node")
publisher = node.create_publisher(DetectedObjectList, "object_detection", 10)

try:
    while True:
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        if not depth_frame:
            continue
        color_frame = aligned_frames.get_color_frame()
        if not color_frame:
            continue
        if num_image < 100:
            num_image += 1
            continue

        # image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        depth_intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics

        boxes_2D = object_2D_detection(color_image)
        num_image += 1
        boxes_2D.sort(key=lambda x: x[1])
        print("box:", boxes_2D)

        num_push_pedal = 0
        num_cup = 0
        num_object = {
            0:0,
            1:0,
            2:0
        }

        detected_object_list = DetectedObjectList()

        for box in boxes_2D:
            if box[5] == 0 and num_object[0] >= 1:
                continue
            lefttop_x, lefttop_y, rightdown_x, rightdown_y = shrink_rect(box[0], box[1], box[2], box[3])
            mask = generate_mask(color_image.shape, lefttop_x, lefttop_y, rightdown_x, rightdown_y)

            target_raw = achieve_targetpointcloud(mask, depth_image, depth_intrinsics)

            target, ind = target_raw.remove_statistical_outlier(nb_neighbors=10, std_ratio=2.0)

            centroid = np.asarray(target.points).mean(axis=0)
            print("The centroid of the ", num_object[box[5]], "st", objectname[box[5]] ,"is:", centroid[0], centroid[1], centroid[2])

            xyz_msg_x = centroid[0]
            xyz_msg_y = centroid[1]
            xyz_msg_z = centroid[2]

            num_object[box[5]] += 1

            object_in_camera = transform_matrix(xyz_msg_x, xyz_msg_y, xyz_msg_z)
            object_in_base = np.dot(camera_to_base, object_in_camera)

            print(objectname[box[5]], ':')
            print(object_in_base)

            detected_object = DetectedObject()
            detected_object.transformation_matrix = object_in_base.ravel().astype(np.float32)
            detected_object.timestamp = str(datetime.now())
            detected_object.category_id = int(box[5])
            # detected_object.category_id = int(box[5]) + 100
            # if box[5] == 2 and (box[2]-box[0])/(box[3]-box[1]) >= 1:
            #     detected_object.category_id += 1  # 门把手横102 竖103
            # print("category_id:",detected_object.category_id)
            detected_object_list.detected_objects.append(detected_object)

        # for box in boxes_2D:
        #     if box[5] == 0:
        #         if num_push_pedal >=1:
        #             continue
        #         lefttop_x, lefttop_y, rightdown_x, rightdown_y = shrink_rect(box[0], box[1], box[2], box[3])
        #         mask = generate_mask(color_image.shape, lefttop_x, lefttop_y, rightdown_x, rightdown_y)
        #
        #         target_raw = achieve_targetpointcloud(mask, depth_image, depth_intrinsics)
        #
        #         target, ind = target_raw.remove_statistical_outlier(nb_neighbors=10, std_ratio=2.0)
        #
        #         centroid = np.asarray(target.points).mean(axis=0)
        #         print("The centroid of the ", num_push_pedal, " push pedal is:", centroid[0], centroid[1], centroid[2])
        #
        #         xyz_msg_x = centroid[0]
        #         xyz_msg_y = centroid[1]
        #         xyz_msg_z = centroid[2]
        #
        #         num_push_pedal += 1
        #
        #         object_in_camera = transform_matrix(xyz_msg_x, xyz_msg_y, xyz_msg_z)
        #         object_in_base = np.dot(camera_to_base, object_in_camera)
        #
        #         print('push pedal:')
        #         print(object_in_base)
        #
        #         detected_object = DetectedObject()
        #         detected_object.transformation_matrix = object_in_base.ravel().astype(np.float32)
        #         detected_object.timestamp = str(datetime.now())
        #         detected_object.category_id = 0
        #         detected_object_list.detected_objects.append(detected_object)
        #
        #     elif box[5] == 1:
        #         lefttop_x, lefttop_y, rightdown_x, rightdown_y = shrink_rect(box[0], box[1], box[2], box[3])
        #         mask = generate_mask(color_image.shape, lefttop_x, lefttop_y, rightdown_x, rightdown_y)
        #
        #         target_raw = achieve_targetpointcloud(mask, depth_image, depth_intrinsics)
        #
        #         target, ind = target_raw.remove_statistical_outlier(nb_neighbors=10, std_ratio=2.0)
        #
        #         centroid = np.asarray(target.points).mean(axis=0)
        #         print("The centroid of the ", num_cup, " cup is:", centroid[0], centroid[1], centroid[2])
        #
        #         xyz_msg_x = centroid[0]
        #         xyz_msg_y = centroid[1]
        #         xyz_msg_z = centroid[2]
        #
        #         num_cup += 1
        #
        #         object_in_camera = transform_matrix(xyz_msg_x, xyz_msg_y, xyz_msg_z)
        #         object_in_base = np.dot(camera_to_base, object_in_camera)
        #
        #         print('cup:')
        #         print(object_in_base)
        #
        #         detected_object = DetectedObject()
        #         detected_object.transformation_matrix = object_in_base.ravel().astype(np.float32)
        #         detected_object.timestamp = str(datetime.now())
        #         detected_object.category_id = 1
        #         detected_object_list.detected_objects.append(detected_object)
        #
        #     elif box[5] == 2:
        #         lefttop_x, lefttop_y, rightdown_x, rightdown_y = shrink_rect(box[0], box[1], box[2], box[3])
        #         mask = generate_mask(color_image.shape, lefttop_x, lefttop_y, rightdown_x, rightdown_y)
        #
        #         target_raw = achieve_targetpointcloud(mask, depth_image, depth_intrinsics)
        #
        #         target, ind = target_raw.remove_statistical_outlier(nb_neighbors=10, std_ratio=2.0)
        #
        #         centroid = np.asarray(target.points).mean(axis=0)
        #         print("The centroid of the ", num_cup, " doorhandle is:", centroid[0], centroid[1], centroid[2])
        #
        #         xyz_msg_x = centroid[0]
        #         xyz_msg_y = centroid[1]
        #         xyz_msg_z = centroid[2]
        #
        #
        #         object_in_camera = transform_matrix(xyz_msg_x, xyz_msg_y, xyz_msg_z)
        #         object_in_base = np.dot(camera_to_base, object_in_camera)
        #
        #         print('doorhandle:')
        #         print(object_in_base)
        #
        #         detected_object = DetectedObject()
        #         detected_object.transformation_matrix = object_in_base.ravel().astype(np.float32)
        #         detected_object.timestamp = str(datetime.now())
        #         detected_object.category_id = 2
        #         detected_object_list.detected_objects.append(detected_object)

        publisher.publish(detected_object_list)

        # # 显示深度图和彩色图
        # cv2.imshow('color', color_image)
        #
        # depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.008), cv2.COLORMAP_JET)
        # cv2.imshow('depth', depth_colormap)
finally:
    node.destroy_node()
    rclpy.shutdown()

    pipeline.stop()






