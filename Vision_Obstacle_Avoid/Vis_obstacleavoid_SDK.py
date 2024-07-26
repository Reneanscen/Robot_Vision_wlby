import os
import numpy as np
import cv2
import open3d as o3d
from datetime import datetime
import torch
from ultralytics import  YOLO
import pyrealsense2 as rs

import rclpy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from rclpy.node import Node

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

def generate_mask(img_shape, pixel_xys):
    mask = np.zeros(img_shape[:2], dtype=np.uint8)
    for pixel_xy in pixel_xys:
        for xy in pixel_xy:
            x, y = xy
            mask[y, x] = 1
    return mask


# 初始化 RealSense 相机
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 15)
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 15)
profile = pipeline.start(config)

align_to = rs.stream.color
align = rs.align(align_to)

rclpy.init()
node = Node("vision_node")
publisher = node.create_publisher(PointCloud2, "Visual_obstacle_avoidance", 10)

# model = torch.hub.load(r'/home/lz/yolov5', 'custom', path=r'runs/train/best_20230921switch.pt', source='local', force_reload=True)

model = YOLO(r'/home/lz/yolov5/runs/v8_obstacle_avoid_best.pt')

num_image = 0

try:
    # 读取并对齐 RGB 和深度图像
    while True:
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()

        if not depth_frame:
            continue
        color_frame = aligned_frames.get_color_frame()

        print(type(color_frame))

        if not color_frame:
            continue
        if num_image < 50:
            num_image += 1
            continue

        # image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        depth_intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics

        results = model(color_image)
        # results.show()

        if results[0].masks is None:
            print('No items in the obstacle list were detected this round！')
            continue

        mask_raw = results[0].masks.data.permute(1, 2, 0).cpu().numpy()

        single_channel_mask = np.any(mask_raw != 0, axis=2).astype(np.uint8) * 255



        # pixel_xys = results[0].masks.xy
        # target_ids = results[0].boxes.cls.tolist()
        # mask = generate_mask(color_image.shape, pixel_xys)
        point_ = achieve_targetpointcloud(single_channel_mask, depth_image, depth_intrinsics)
        pointcloud_data = np.asarray(point_.points)
        target_id = 1

        pcl_msg = PointCloud2()

        pcl_msg.header.stamp = node.get_clock().now().to_msg()
        pcl_msg.header.frame_id = "Realsense Camera_link"

        pcl_msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            # 添加其他字段，如果有的话
        ]

        pcl_msg.point_step = 12

        pcl_msg.width = pointcloud_data.shape[0]  # 点云数据的数量
        pcl_msg.height = 1  # 1D点云

        pcl_msg.data = pointcloud_data.astype(np.float32).tobytes()

        publisher.publish(pcl_msg)

        # 可视化点云
        # o3d.visualization.draw_geometries([point_])

finally:
    node.destroy_node()
    rclpy.shutdown()
    pipeline.stop()