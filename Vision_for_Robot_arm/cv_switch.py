import torch
from ultralytics import YOLO
import cv2
import numpy as np
import open3d as o3d
import pyrealsense2 as rs
import math

import rclpy
from rclpy.node import Node

from robot_cv_msg.msg import Matrix4x4, GraspInfo, MultiObjGraspInfo
from datetime import datetime

# Load a pretrained YOLO model (recommended for training)
model = YOLO('/home/lz/yolov5/runs/train/best_20230921switch.pt')


objectname = {
    0:"one_switch_on",
    1:"one_switch_off",
    2:"two_switch",
    3:"three_switch",
    4:"branch_switch_on",
    5:"branch_switch_off"
}

camera_to_base = np.array([[0.4830191777293635, -0.5454382934019704, 0.6849741177857716, 0.09830437700902608],
                           [-0.3508700489363267, -0.8372981068953109, -0.4193114462412742, 0.5652720778206468],
                           [0.8022360517360645, -0.03780143225164029, -0.5958090038046218, 0.794058946750924],
                           [0.0, 0.0, 0.0, 1.0]])

camera_to_eef = np.array([[-0.6593257364456375, 0.7518566744191698, -0.0010556475571349665, -0.04889370549206029],
                          [-0.7516566700596122, -0.6591166039133254, 0.02403232824052836, 0.08712456749635022],
                          [0.017373071556685354, 0.01663861704324228, 0.9997106245347074, 0.12415632018767313],
                          [0.0, 0.0, 0.0, 1.0]])
eef_to_base = np.array([[ 0.340221, 0.706294, 0.620804, 0.137491],
    [0.823234, -0.542779,  0.166365, 0.0368475],
    [0.454462, 0.454466,  -0.76611,  0.732583],
    [0,         0,         0,          1]])

def object_2D_detection(img):
    # Inference
    results = model(img, imgsz=1280)

    boxes_ = results[0].boxes.xyxy.tolist()
    cls_ = results[0].boxes.cls.tolist()
    boxes = [boxes_ + [cls_] for boxes_, cls_ in zip(boxes_, cls_)]

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


def judge_which_switch(box, boxes):
    if len(boxes) == 0:
        return -1
    x1, y1, x2, y2, cls = box

    for other_box in boxes:
        other_x1, other_y1, other_x2, other_y2, other_cls = other_box

        # 排除一下自身
        if (x1 == other_x1 and x2 == other_x2 and y1 == other_y1 and y2 == other_y2):
            continue

        # 检查两个边界框是否重叠
        if (x1 < other_x2 and x2 > other_x1 and y1 < other_y2 and y2 > other_y1):
            return other_cls

    # 没有找到重叠的边界框
    return -1


# 初始化 RealSense 相机
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 15)
config.enable_stream(rs.stream.color, 1280, 720, rs.format.rgb8, 15)
profile = pipeline.start(config)

align_to = rs.stream.color
align = rs.align(align_to)

rclpy.init()
node = Node("vision_node")
publisher = node.create_publisher(MultiObjGraspInfo, "object_detection", 10)

num_image = 0

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
        # boxes_2D.sort(key=lambda x: x[1])
        # print("box:", boxes_2D)

        num_push_pedal = 0
        num_cup = 0
        num_object = {
            0:0,
            1:0,
            2:0,
            3:0,
            4:0,
            5:0
        }

        detected_object_list = MultiObjGraspInfo()

        for box in boxes_2D:
            if box[4] == 0 or box[4] == 4:
                lefttop_x, lefttop_y, rightdown_x, rightdown_y = shrink_rect(box[0], box[1], box[2], box[3])
                mask = generate_mask(color_image.shape, lefttop_x, lefttop_y, rightdown_x, int(lefttop_y+(rightdown_y-lefttop_y)*0.33))

                target_raw = achieve_targetpointcloud(mask, depth_image, depth_intrinsics)

                target, ind = target_raw.remove_statistical_outlier(nb_neighbors=10, std_ratio=2.0)

                centroid = np.asarray(target.points).mean(axis=0)
                print("The centroid of the ", num_object[box[5]], "st", objectname[box[5]] ,"is:", centroid[0], centroid[1], centroid[2])

                xyz_msg_x = centroid[0]
                xyz_msg_y = centroid[1]
                xyz_msg_z = centroid[2]
            elif box[4] == 1 or box[4] == 5:
                lefttop_x, lefttop_y, rightdown_x, rightdown_y = shrink_rect(box[0], box[1], box[2], box[3])
                mask = generate_mask(color_image.shape, lefttop_x, int(lefttop_y+(rightdown_y-lefttop_y)*0.66), rightdown_x, rightdown_y)

                target_raw = achieve_targetpointcloud(mask, depth_image, depth_intrinsics)

                target, ind = target_raw.remove_statistical_outlier(nb_neighbors=10, std_ratio=2.0)

                centroid = np.asarray(target.points).mean(axis=0)
                print("The centroid of the ", num_object[box[5]], "st", objectname[box[5]], "is:", centroid[0],
                      centroid[1], centroid[2])

                xyz_msg_x = centroid[0]
                xyz_msg_y = centroid[1]
                xyz_msg_z = centroid[2]
            elif box[4] == 2 or box[4] == 3:
                continue

            num_object[box[5]] += 1

            object_in_camera = transform_matrix(xyz_msg_x, xyz_msg_y, xyz_msg_z)
            temp_object_in_eff = np.dot(camera_to_eef, object_in_camera)
            object_in_base = np.dot(eef_to_base, temp_object_in_eff)
            # object_in_base = np.dot(camera_to_base, object_in_camera)

            print(objectname[box[5]], ':')
            print(object_in_base)

            matrix = Matrix4x4()
            matrix.elements = object_in_base.ravel().astype(np.float32)

            detected_object = GraspInfo()
            detected_object.grasp_matrices.append(matrix)
            detected_object.target_id = int(box[5])
            # detected_object.category_id = int(box[5]) + 100
            # if box[5] == 2 and (box[2]-box[0])/(box[3]-box[1]) >= 1:
            #     detected_object.category_id += 1  # 门把手横102 竖103
            # print("category_id:",detected_object.category_id)
            detected_object_list.info_list.append(detected_object)
        publisher.publish(detected_object_list)
finally:
    node.destroy_node()
    rclpy.shutdown()
    pipeline.stop()


