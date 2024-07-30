#!/usr/bin/env python
# -*- coding: UTF-8 -*-
"""
@Project : aloha_data_collection
@File    : data_record.py
@IDE     : PyCharm
@Author  : bhuang
@Date    : 2024/4/7 10:29:13
"""

import cv2
import time
import h5py
import numpy as np
from pathlib import Path

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from message_filters import Subscriber, ApproximateTimeSynchronizer

from realsense_camera import RealsenseCamera
from motion_control_interfaces.msg import SlaveArmStatus
from motion_control_interfaces.msg import TeleoperationJointControl


class DataRecordNode(Node):
    def __init__(self, configs):
        super(DataRecordNode, self).__init__(configs["node_name"])
        master_joint_topic = configs.get("master_joint_topic", "master_joint_topic")
        slave_joint_topic = configs.get("slave_joint_topic", "slave_joint_topic")
        camera_fps = configs.get("camera_fps", 60)
        image_height = configs.get("image_height", 480)
        camera_serials = configs["camera_serials"]
        self.save_dir = Path(configs.get("save_dir", "./dataset"))
        self.local_mode = configs.get("local_mode", True)
        self.camera_names = configs["camera_names"]
        self.joint_num = configs.get("joint_num", 8)
        self.master_gripper_position_open = configs.get("master_gripper_position_open")
        self.master_gripper_position_close = configs.get("master_gripper_position_close")
        self.slave_gripper_position_open = configs.get("slave_gripper_position_open")
        self.slave_gripper_position_close = configs.get("slave_gripper_position_close")

        self.master_joint_sub = Subscriber(self, TeleoperationJointControl, master_joint_topic)   # FIXME: change to master topic
        self.slave_joint_sub = Subscriber(self, SlaveArmStatus, slave_joint_topic)
        if self.local_mode:
            self.right_wrist_camera = RealsenseCamera(resulution=image_height, fps=camera_fps, device_id=camera_serials[0])
            self.head_camera = RealsenseCamera(resulution=image_height, fps=camera_fps, device_id=camera_serials[1])
        else:
            self.right_wrist_sub = Subscriber(self, CompressedImage, self.camera_names[0])
            self.head_camera_sub = Subscriber(self, CompressedImage, self.camera_names[1])

        if self.local_mode:
            self.ts = ApproximateTimeSynchronizer([self.master_joint_sub, self.slave_joint_sub], 10, 0.01)
            self.ts.registerCallback(self._callback_local)
        else:
            self.ts = ApproximateTimeSynchronizer([self.master_joint_sub, self.slave_joint_sub, self.head_camera_sub, self.right_wrist_sub], 10, 0.01)
            self.ts.registerCallback(self._callback)

        self.recording = False
        self.start_id = self.get_start_id()
        self.cv_bridge = CvBridge()
        self.time0 = time.time()
        self.cnt = 0

        self._init_record()
        self.get_logger().info(f"press `Enter` to start recording, press `Enter` again to end recording, "
                               f"press `ESC` to exit, press `Q` to terminal this time recording")

    def _init_record(self):
        self.episodes = {
            # "/base_action": [],
            "/action": [],
            "/observations/qpos": [],
            "/observations/qvel": [],
            # "/observations/images/right_wrist": [],
            # "/observations/images/top": []
        }
        self.episodes.update({
            f"/observations/images/{camera_name}": [] for camera_name in self.camera_names
        })
        self.recording = False

    def _callback(self, master_joint_msg: TeleoperationJointControl,
                  slave_joint_msg: SlaveArmStatus,
                  head_image_msg: CompressedImage,
                  right_wrist_msg: CompressedImage):
        master_joints, master_velocities = self._joints_msg_to_list_master(master_joint_msg)
        slave_joints, slave_velocities = self._joints_msg_to_list_slave(slave_joint_msg)
        head_image = self.cv_bridge.compressed_imgmsg_to_cv2(head_image_msg)
        right_wrist_image = self.cv_bridge.compressed_imgmsg_to_cv2(right_wrist_msg)

        self._record_data(master_joints, slave_joints, slave_velocities, head_image, right_wrist_image)

    def _callback_local(self, master_joint_msg: TeleoperationJointControl, slave_joint_msg: SlaveArmStatus):
        master_joints, master_velocities = self._joints_msg_to_list_master(master_joint_msg)
        slave_joints, slave_velocities = self._joints_msg_to_list_slave(slave_joint_msg)
        head_image = self.head_camera.get_image()
        right_wrist_image = self.right_wrist_camera.get_image()

        self._record_data(master_joints, slave_joints, slave_velocities, head_image, right_wrist_image)

    def _joints_msg_to_list_master(self, joint_msg: TeleoperationJointControl):
        joint_positions = [joint.joint_position for joint in joint_msg.joints]
        # norm gripper position
        joint_positions[-1] = (joint_positions[-1] / 1000.0 - self.master_gripper_position_close) / (self.master_gripper_position_open - self.master_gripper_position_close)
        joint_speeds = [joint.joint_speed for joint in joint_msg.joints]   # FIXME: not velocity
        return joint_positions, joint_speeds

    def _joints_msg_to_list_slave(self, joint_msg: SlaveArmStatus):
        if joint_msg.error_code == 1:
            return None, None
        else:
            joint_positions = [joint.joint_position for joint in joint_msg.joints]
            # norm gripper position
            joint_positions[-1] = (joint_positions[-1] / 1000.0 - self.slave_gripper_position_close) / (self.slave_gripper_position_open - self.slave_gripper_position_close)
            joint_speeds = [joint.joint_speed for joint in joint_msg.joints]  # FIXME: not velocity
            return joint_positions, joint_speeds

    def _record_data(self, master_joints, slave_joints, slave_velocities, head_image, right_wrist_image):
        if self.recording and master_joints is not None and slave_joints is not None:
            self.episodes["/action"].append(master_joints)
            self.episodes["/observations/qpos"].append(slave_joints)
            self.episodes["/observations/qvel"].append(slave_velocities)  # FIXME: WARNNING: dummy velocity

            ret1, encode_right_wrist = cv2.imencode(".jpg", right_wrist_image.copy())
            ret2, encode_head_image = cv2.imencode(".jpg", head_image.copy())
            self.episodes[f"/observations/images/{self.camera_names[0]}"].append(encode_right_wrist)
            self.episodes[f"/observations/images/{self.camera_names[1]}"].append(encode_head_image)

        show_image = np.concatenate([head_image, right_wrist_image], axis=1)
        self.cnt += 1
        cv2.putText(show_image, f"FPS: {self.cnt / (time.time() - self.time0):.2f}", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 1)
        cv2.imshow("press `Enter` to start recording, press `Enter` again to end recording, "
                   "press `ESC` to exit, press `Q` to terminal this time recording", show_image)
        key = cv2.waitKey(1)
        if key == 27:       # ESC
            exit(0)
        elif key == ord('q'):   # Q
            self._init_record()
            self.get_logger().info("end recording and data not saved.")
        elif key == 13:     # ENTER
            if self.recording:
                self.get_logger().info("end recording.")
                self.save()
                self.recording = False
            else:
                self.get_logger().info("start recording...")
                self.recording = True

    def save(self):
        self.get_logger().info("recorded data saving...")
        max_timesteps = len(self.episodes["/action"])

        file_name = self.save_dir / f"episode_{self.start_id}.hdf5"
        with h5py.File(file_name, 'w', rdcc_nbytes=1024 ** 2 * 2) as root:
            root.attrs['sim'] = False
            root.attrs['compress'] = True
            obs = root.create_group('observations')
            image = obs.create_group('images')
            for cam_name in self.camera_names:
                max_len = max([len(d) for d in self.episodes[f"/observations/images/{cam_name}"]])
                image.create_dataset(cam_name, (max_timesteps, max_len), dtype='uint8',
                                     chunks=(1, max_len), )

                # pad
                padded_image_list = []
                compressed_images = self.episodes[f"/observations/images/{cam_name}"]
                for compressed_image in compressed_images:
                    padded_image = np.zeros(max_len, dtype=np.uint8)
                    padded_image[:len(compressed_image)] = compressed_image
                    padded_image_list.append(padded_image)
                self.episodes[f"/observations/images/{cam_name}"] = padded_image_list

            obs.create_dataset('qpos', (max_timesteps, self.joint_num))
            obs.create_dataset('qvel', (max_timesteps, self.joint_num))
            root.create_dataset('action', (max_timesteps, self.joint_num))
            # root.create_dataset('action', (max_timesteps, 7))

            for name, array in self.episodes.items():
                root[name][...] = array
        self.get_logger().info(f"recorded data saved in {str(file_name)}")
        self.start_id += 1
        self._init_record()

    def get_start_id(self):
        if not self.save_dir.exists():
            self.save_dir.mkdir(parents=True, exist_ok=True)
            return 0

        file_ids = [int(str(file).split("episode_")[-1].split(".")[0]) for file in self.save_dir.glob("*.hdf5")]
        if len(file_ids) > 0:
            return max(file_ids) + 1
        else:
            return 0


def main(cfg):
    rclpy.init()
    node = DataRecordNode(cfg)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    import yaml

    config = yaml.safe_load(open("configs/right_arm_data_record.yaml", 'r'))
    main(config)
