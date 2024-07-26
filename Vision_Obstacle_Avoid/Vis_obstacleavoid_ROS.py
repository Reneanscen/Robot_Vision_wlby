import os
import numpy as np
import cv2
import open3d as o3d
from datetime import datetime
import torch
from ultralytics import  YOLO

import rclpy
from sensor_msgs.msg import PointCloud2, PointField, Image, CameraInfo

from cv_bridge import CvBridge
from std_msgs.msg import Header
from rclpy.node import Node

def depth_to_pointcloud(depth_image, fx, fy, cx, cy):
    # Create Open3D Image from depth map
    o3d_depth = o3d.geometry.Image(depth_image)

    # Get intrinsic parameters
    # fx, fy, cx, cy = intrinsic.fx, intrinsic.fy, intrinsic.ppx, intrinsic.ppy

    # Create Open3D PinholeCameraIntrinsic object
    o3d_intrinsic = o3d.camera.PinholeCameraIntrinsic(width=depth_image.shape[1], height=depth_image.shape[0], fx=fx, fy=fy, cx=cx, cy=cy)

    # Create Open3D PointCloud object from depth image and intrinsic parameters
    pcd = o3d.geometry.PointCloud.create_from_depth_image(o3d_depth, o3d_intrinsic)

    return pcd

def achieve_targetpointcloud(mask, depth, fx, fy, cx, cy):
    mask_resized = cv2.resize(mask, (depth.shape[1], depth.shape[0]))
    roi = cv2.bitwise_and(depth, depth, mask=mask_resized)
    target_pointcloud = depth_to_pointcloud(roi, fx, fy, cx, cy)
    return target_pointcloud

def generate_mask(img_shape, pixel_xys):
    mask = np.zeros(img_shape[:2], dtype=np.uint8)
    for pixel_xy in pixel_xys:
        for xy in pixel_xy:
            x, y = xy
            mask[y, x] = 1
    return mask

class RealSenseSubscriber(Node):
    def __init__(self):
        super().__init__('realsense_subscriber')
        self.bridge = CvBridge()

        # 创建 RGB 和深度图像的订阅对象
        self.rgb_subscription = self.create_subscription(Image, '/camera/camera/color/image_raw', self.get_rgb_image, 10)
        self.depth_subscription = self.create_subscription(Image, '/camera/camera/depth/image_rect_raw', self.get_depth_image, 10)

        # 创建相机内参的订阅对象
        self.rgb_info_subscription = self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self.get_rgb_info, 10)
        self.depth_info_subscription = self.create_subscription(CameraInfo, '/camera/camera/depth/camera_info', self.get_depth_info, 10)

        # 初始化 RGB 和深度图像的数据
        self.rgb_image_data = None
        self.depth_image_data = None

        # 初始化相机内参的数据
        self.rgb_info_data = None
        self.depth_info_data = None

    def get_rgb_image(self, msg):
        # 获取 RGB 图像数据
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.rgb_image_data = cv_image

    def get_depth_image(self, msg):
        # 获取深度图像数据
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.depth_image_data = cv_image

    def get_rgb_info(self, msg):
        # 获取 RGB 相机内参数据
        self.rgb_info_data = msg

    def get_depth_info(self, msg):
        # 获取深度相机内参数据
        self.depth_info_data = msg



rclpy.init()
node_spatial_publish = Node("vision_node")
print(3333333333333333)
node = RealSenseSubscriber()
publisher = node_spatial_publish.create_publisher(PointCloud2, "Visual_obstacle_avoidance", 10)

# model = torch.hub.load(r'/home/lz/yolov5', 'custom', path=r'runs/train/best_20230921switch.pt', source='local', force_reload=True)

# model = YOLO(r'/home/lz/yolov5/runs/v8_obstacle_avoid_best.pt')
model = YOLO(r'/home/zme/Spatial_AI/v8_obstacle_avoid_best.pt')
print(2222222222222222)

try:
    # 读取并对齐 RGB 和深度图像
    while True:
        print(99999999999999999999999999999999999)
        if node.rgb_image_data is not None and node.depth_image_data is not None:
            color_image = node.rgb_image_data
            print("color_image:", type(color_image))
            depth_image = node.depth_image_data
            print("depth_image:", type(depth_image))
            print(11111111111111111)


            # fx, fy, cx, cy = node_subscriber.depth_info_data.K[0], node_subscriber.depth_info_data.K[4],node_subscriber.depth_info_data.K[2],node_subscriber.depth_info_data.K[5]
            # fx, fy, cx, cy = 386.39, 386.39, 326.04, 235.52
            fx, fy, cx, cy = 912.02, 912.34, 653.86, 372.13

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
            point_ = achieve_targetpointcloud(single_channel_mask, depth_image, fx, fy, cx, cy)
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
        if node.rgb_info_data is not None and node.depth_info_data is not None:
            # 在这里添加对相机内参的处理代码
            node.get_logger().info('RGB Camera Info: %s' % node.rgb_info_data)
            node.get_logger().info('Depth Camera Info: %s' % node.depth_info_data)
            print(1010101010101010101010101)
            # print(node.depth_info_data.k)
            print(1010101010101010101010101)
        rclpy.spin_once(node)
finally:
    node.destroy_node()
    node_spatial_publish.destroy_node()
    rclpy.shutdown()