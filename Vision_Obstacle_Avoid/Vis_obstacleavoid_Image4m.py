import os
import numpy as np
import cv2
import open3d as o3d
from datetime import datetime
import torch
from ultralytics import  YOLO

import rclpy
from zme_msg_srv.msg import Obstacleavoid, ObstacleavoidList
from sensor_msgs.msg import PointCloud2, PointField, Image, CameraInfo

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from custom_image_msg.msg import Image4m, String

from cv_bridge import CvBridge
from std_msgs.msg import Header
from rclpy.node import Node

import array

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


        self.filename = f"images_001"
        self.counter = 0


        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=50)

        # 创建 RGB 和深度图像的订阅对象
        self.rgb_subscription = self.create_subscription(Image4m, '/head_cam_color', self.get_rgb_image, qos_profile=self.qos_profile)
        self.depth_subscription = self.create_subscription(Image4m, '/head_cam_depth', self.get_depth_image, qos_profile=self.qos_profile)

        # 创建相机内参的订阅对象
        # self.rgb_info_subscription = self.create_subscription(CameraInfo, '/camera/color/camera_info', self.get_rgb_info, 10)
        # self.depth_info_subscription = self.create_subscription(CameraInfo, '/camera/depth/camera_info', self.get_depth_info, 10)

        # 初始化 RGB 和深度图像的数据
        self.rgb_image_data = None
        self.depth_image_data = None

        # # 初始化相机内参的数据
        # self.rgb_info_data = None
        # self.depth_info_data = None

    def get_rgb_image(self, msg):
        # 获取 RGB 图像数据
        help_image_msg = Image()
        help_image_msg.step = msg.step
        length = msg.height * msg.step
        help_image_msg.data = array.array('B')
        help_image_msg.data.frombytes(msg.data[:length].tobytes())
        help_image_msg.encoding = 'bgr8'
        help_image_msg.height = msg.height
        help_image_msg.width = msg.width
        help_image_msg.is_bigendian = msg.is_bigendian


        cv_image = self.bridge.imgmsg_to_cv2(help_image_msg, desired_encoding='passthrough')
        self.rgb_image_data = cv_image

        # self.filename = f"image_{self.counter}.png"
        # self.counter += 1
        #
        # rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        #
        # cv2.imwrite(self.filename, rgb_image)

    def get_depth_image(self, msg):
        # 获取深度图像数据
        # 获取 RGB 图像数据
        help_image_msg = Image()
        help_image_msg.step = msg.step
        length = msg.height * msg.step
        help_image_msg.data = array.array('B')
        help_image_msg.data.frombytes(msg.data[:length].tobytes())
        help_image_msg.encoding = 'bgr8'
        help_image_msg.height = msg.height
        help_image_msg.width = msg.width
        help_image_msg.is_bigendian = msg.is_bigendian

        cv_image = self.bridge.imgmsg_to_cv2(help_image_msg, desired_encoding='passthrough')

        hlr = cv2.split(cv_image)
        h = hlr[0]
        l = hlr[1]

        # 转换为16位单通道
        h_u16 = h.astype(np.uint16)
        l_u16 = l.astype(np.uint16)
        image_16bit_single_channel = h_u16 * 256 + l_u16

        self.depth_image_data = image_16bit_single_channel

        # self.filename = f"depth_{self.counter}.png"
        # self.counter += 1
        # depth_colormap = cv2.applyColorMap \
        #     (cv2.convertScaleAbs(image_16bit_single_channel, alpha=0.008)
        #      , cv2.COLORMAP_JET)
        #
        # cv2.imwrite(self.filename, depth_colormap)

    # def get_rgb_info(self, msg):
    #     # 获取 RGB 相机内参数据
    #     self.rgb_info_data = msg
    #
    # def get_depth_info(self, msg):
    #     # 获取深度相机内参数据
    #     self.depth_info_data = msg

rclpy.init()
node_spatial_publish = Node("vision_node")
node = RealSenseSubscriber()
publisher = node_spatial_publish.create_publisher(ObstacleavoidList, "Visual_obstacle_avoidance", 10)

model = YOLO(r'/home/zme/slam_group/ground_obstacle/v8_obstacle_avoid_best.pt')

fx, fy, cx, cy = 912.7135, 912.5785, 653.788, 364.5224

frame_id = "Realsense Camera_link"


try:
    # while rclpy.ok():
    #     if node.rgb_info_data is not None and node.depth_info_data is not None:
    #         fx, fy, cx, cy = node.depth_info_data.k[0], node.depth_info_data.k[4], node.depth_info_data.k[2], node.depth_info_data.k[5]
    #         # frame_id = node.rgb_info_data.frame_id
    #         break
    #     rclpy.spin_once(node)

    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.5)
        if node.rgb_image_data is not None and node.depth_image_data is not None:
            color_image = node.rgb_image_data
            depth_image = node.depth_image_data

            results = model(color_image)

            if results[0].masks is None:
                print('No items in the obstacle list were detected this round！')
                continue

            obstacleList = ObstacleavoidList()

            mask_raws = results[0].masks.data.permute(1, 2, 0).cpu().numpy()
            maks_raws_shape = mask_raws.shape

            label = results[0].boxes.cls.tolist()

            for s in range(len(label)):
                single_channel_mask = np.zeros((maks_raws_shape[0], maks_raws_shape[1]))
                for i in range(maks_raws_shape[0]):
                    for j in range(maks_raws_shape[1]):
                        if mask_raws[i, j, s] != 0:
                            single_channel_mask[i, j] = 1

                single_channel_mask = np.any(mask_raws != 0, axis=2).astype(np.uint8) * 255

                point_ = achieve_targetpointcloud(single_channel_mask, depth_image, fx, fy, cx, cy)
                pointcloud_data = np.asarray(point_.points)
                target_id = int(label[s])

                pcl_msg = PointCloud2()

                pcl_msg.header.stamp = node.get_clock().now().to_msg()
                pcl_msg.header.frame_id = frame_id

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

                obstacle = Obstacleavoid()
                obstacle.point_cloud = pcl_msg
                obstacle.class_id = target_id

                obstacleList.obstacle.append(obstacle)

            publisher.publish(obstacleList)

            # 可视化点云
            # o3d.visualization.draw_geometries([point_])
finally:
    node.destroy_node()
    node_spatial_publish.destroy_node()
    rclpy.shutdown()