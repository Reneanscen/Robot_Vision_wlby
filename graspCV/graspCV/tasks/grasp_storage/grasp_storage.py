import cv2
import torch
import numpy as np
import open3d as o3d
from numpy import ndarray
from pyquaternion import Quaternion
from typing import Union, Tuple, List, Dict

from ultralytics import YOLO
from ultralytics.data.augment import LetterBox
from ultralytics.utils.ops import process_mask, scale_boxes, scale_image

from core import TaskInterface
from core.logger import logger
from .utils import non_max_suppression
from configs import REAL_SENSE_D435i_CAM_INTRINSIC


class GraspStorage(TaskInterface):
    def __init__(self, model_path: str,
                 input_shape: Union[Tuple, List] = (640, 640),  # [height, width]
                 backend: str = "tensorrt",
                 target_map: Dict = None,
                 debug: bool = False,
                 score_thresh: float = 0.5,
                 iou_thresh: float = 0.45):
        super(GraspStorage, self).__init__(model_path, input_shape, backend,
                                           target_map, debug)

        self.score_thresh = score_thresh
        self.iou_thresh = iou_thresh
        self.stride = 32

        # intrinsic matrix: realsense D435i 1280x720
        # focal = [909.62, 908.809, 643.526, 362.554]  # [fx, fy, u0, v0]
        # self.camera_matrix = np.array([[focal[0], 0, focal[2]],
        #                                [0, focal[1], focal[3]],
        #                                [0, 0, 1]], dtype=np.float32)
        self.camera_matrix = REAL_SENSE_D435i_CAM_INTRINSIC
        # self.camera_matrix_inv = np.linalg.inv(self.camera_matrix)
        # extrinsic matrix: camera to hand
        self.camera_to_hand = np.array([[-0.6823429871800, 0.73093423844425, 0.011966073546278, -0.0462806520892],
                                        [-0.7308176858390, -0.6824478828560, 0.013053629769688, 0.08911593726022],
                                        [0.01770756649239, 0.00016203455292, 0.999843195623054, 0.12139120123897],
                                        [0.0, 0.0, 0.0, 1.0]])

    def _preprocess(self, image: ndarray) -> ndarray:
        if self.backend == "pytorch":
            return image

        img = LetterBox(self.input_shape, auto=False, stride=self.stride)(image=image)
        img = np.expand_dims(img, axis=0)
        img = img[..., ::-1].transpose((0, 3, 1, 2))  # BGR to RGB, BHWC to BCHW, (n, 3, h, w)
        img = np.ascontiguousarray(img)  # contiguous
        img /= 255  # 0 - 255 to 0.0 - 1.0
        return img

    def _load_model_pytorch(self):
        return YOLO(self.model_path)

    def _infer_pytorch(self, image: ndarray, **kwargs):
        return self.model(image, conf=self.score_thresh, iou=self.iou_thresh)[0]

    def _post_process(self, outputs, **kwargs) -> Union[List, bool]:
        image = kwargs.get("image")
        ih, iw = image.shape[:2]
        depth = kwargs.get("depth")
        task_id = kwargs.get("task_id")
        target_id = kwargs.get("target_id")
        R_matrix = kwargs.get("robot_extrinsic")  # noqa

        if self.backend == "pytorch":
            bboxes = outputs.boxes.xyxy.cpu().numpy()  # [N, 4]
            if bboxes.shape[0] == 0:  # don't detect shoes
                return []
            scores = outputs.boxes.conf.cpu().numpy()  # [N]
            classes = outputs.boxes.cls.cpu().numpy()  # [N]
            masks = scale_image(outputs.masks.data.permute(1, 2, 0).cpu().numpy(), (ih, iw))
        else:
            p = non_max_suppression(outputs[0],
                                    self.score_thresh,
                                    self.iou_thresh,
                                    agnostic=False,
                                    max_det=300,
                                    nc=len(self.model.names))

            pred = p[0]  # remove batch
            if pred.shape[0] == 0:
                return []

            bboxes = pred[:, :4]
            scores = pred[:, 4]
            classes = pred[:, 5].astype(int)
            mask_preds = pred[:, 6:]

            # masks = process_mask(torch.from_numpy(outputs[1][0]), torch.from_numpy(mask_preds),
            #                      torch.from_numpy(bboxes), self.input_shape, upsample=True)
            # masks = scale_image(masks.permute(1, 2, 0).cpu().numpy(), (ih, iw))
            # bboxes = scale_boxes(self.input_shape, bboxes, (ih, iw))

            masks = process_mask(outputs[1][0], mask_preds, bboxes, self.input_shape, upsample=True)
            masks = scale_image(masks.permute(1, 2, 0).numpy(), (ih, iw))
            bboxes = scale_boxes(self.input_shape, bboxes, (ih, iw)).numpy()

        results = []
        # R_matrix = np.eye(3, dtype=np.float32)
        for i, bbox in enumerate(bboxes):
            mask = masks[:, :, i]  # [ih, iw]
            score = scores[i]  # [1]
            cls = classes[i]  # [1]

            # filter objects according to target_id
            flag = False
            if target_id == 11 and cls == 11:  # clothes
                flag = True
            elif target_id == 21 and cls == 12:  # plastic_toy
                flag = True
            elif target_id == 22 and cls == 13:  # plush_toy
                flag = True
            elif target_id == 0 and cls in [11, 12, 13]:  # storage all things
                flag = True
            if not flag:
                continue

            is_soft = cls in [11, 13]
            is_plastic = cls in [12]

            if is_soft:
                target, _ = self.create_target_point_cloud(mask, depth).remove_statistical_outlier(
                    nb_neighbors=10, std_ratio=2.0)
                target_points = np.asarray(target.points)
                centroid_xys = self.calc_high_points(target_points, min_distance=0.1)
                for centroid_xy in centroid_xys:
                    max_z_point, points_in_range = self.calc_low_point_in_range(target_points, centroid_xy,
                                                                                max_range=0.1)
                    centroid = [centroid_xy[0], centroid_xy[1], (max_z_point[2] + centroid_xy[2]) / 2]
                    centroid_4d = np.ones((4, 1), dtype=np.float32)  # [4, 1] (x, y, z, 1)
                    centroid_4d[:3] = centroid

                    angle = self.calc_grasp_angle(np.asarray(points_in_range), centroid_xy)
                    q = Quaternion(axis=[0, 0, 1], degrees=angle).rotation_matrix
                    rt = np.eye(4, dtype=np.float32)
                    rt[:3, :3] = q
                    rt[:, 3:4] = centroid_4d
                    rt = np.dot(R_matrix, rt)
                    # rt[:3, :3] = np.dot(R_matrix, q)
                    # rt[:, 3:4] = np.dot(self.camera_to_hand, centroid_4d)

                    result = {"target_id": self.target_map[cls], "grasp_scores": [float(score)], "grasp_rt": [rt]}
                    results.append(result)

            if is_plastic:
                quantiles, endpoints, angle = self.calc_quantiles_angle(mask)

                quantiles_3d = []
                for quantile in quantiles:
                    quantile_3d = self.pixel_to_3d(depth, quantile[0], quantile[1])
                    quantiles_3d.append(quantile_3d)

                endpoints_3d_l = self.pixel_to_3d(depth, endpoints[0][0], endpoints[0][1])
                endpoints_3d_r = self.pixel_to_3d(depth, endpoints[1][0], endpoints[1][1])

                if np.linalg.norm(endpoints_3d_l - endpoints_3d_r) <= 0.06:
                    for quantile_3d in quantiles_3d:
                        centroid_x, centroid_y, centroid_z = quantile_3d[:, 0]
                        centroid = [centroid_x, centroid_y, centroid_z]
                        centroid_4d = np.ones((4, 1), dtype=np.float32)  # [4, 1] (x, y, z, 1)
                        centroid_4d[:3] = centroid

                        q = Quaternion(axis=[0, 0, 1], degrees=angle).rotation_matrix
                        rt = np.eye(4, dtype=np.float32)
                        rt[:3, :3] = q
                        rt[:, 3:4] = centroid_4d
                        rt = np.dot(R_matrix, rt)
                        # rt[:3, :3] = np.dot(R_matrix, q)
                        # rt[:, 3:4] = np.dot(self.camera_to_hand, centroid_4d)

                        result = {"target_id": self.target_map[cls], "grasp_scores": [float(score)],
                                  "grasp_rt": [rt], "loc_type": 1}
                        results.append(result)
                else:
                    logger.warning("The size of the plastic toy is too big")

            if self.debug:
                contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                cv2.drawContours(image, contours, -1, (0, 0, 255), 2)
                cv2.putText(image, f"{score:.3f} {cls}", tuple(bbox[:2].astype(int)), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                            (0, 0, 255), 2)

        if self.debug:
            cv2.imshow('image', image)
            cv2.waitKey()

        return results

    def create_target_point_cloud(self, mask, depth):
        fx = self.camera_matrix[0][0]
        fy = self.camera_matrix[1][1]
        ppx = self.camera_matrix[0][2]
        ppy = self.camera_matrix[1][2]

        mask_resized = cv2.resize(mask, (depth.shape[1], depth.shape[0]))
        roi = cv2.bitwise_and(depth, depth, mask=mask_resized)

        o3d_depth = o3d.geometry.Image(roi)
        o3d_intrinsic = o3d.camera.PinholeCameraIntrinsic(width=roi.shape[1], height=roi.shape[0],
                                                          fx=fx, fy=fy, cx=ppx, cy=ppy)
        target_point_cloud = o3d.geometry.PointCloud.create_from_depth_image(o3d_depth, o3d_intrinsic)

        return target_point_cloud

    @staticmethod
    def calc_high_points(points, min_distance=0.1):
        z_indices = np.argsort(points[:, 2])
        largest_z_points = points[z_indices]

        selected_points = []
        for point in largest_z_points:
            if not any(np.linalg.norm(point[:2] - selected_point[:2]) < min_distance for selected_point in
                       selected_points):
                selected_points.append(point)

            if len(selected_points) == 3:
                break

        return selected_points

    @staticmethod
    def calc_low_point_in_range(points, min_z_point, max_range=0.1):
        points_in_range = points[
            (np.abs(points[:, 0] - min_z_point[0]) <= max_range) &
            (np.abs(points[:, 1] - min_z_point[1]) <= max_range)
            ]

        if len(points_in_range) > 0:
            max_z_index = np.argmax(points_in_range[:, 2])
            max_z_point = points_in_range[max_z_index]
            return max_z_point, points_in_range
        else:
            return None

    @staticmethod
    def calc_grasp_angle(points, min_z_point):
        """
        织物和软质玩具抓取角度
        """

        def sample_points(points, num_samples=100):
            if len(points) <= num_samples:
                return np.array(points)
            else:
                sampled_indices = np.random.choice(len(points), num_samples, replace=False)
                sampled_points = np.array(points)[sampled_indices]
                return sampled_points

        def calc_z_diff_over_xy(points, fixed_point):
            z_diff = np.abs(points[:, 2] - fixed_point[2])
            x_y_length = np.linalg.norm(points[:, :2] - fixed_point[:2], axis=1)
            return np.mean(z_diff / (x_y_length + 1e-5))

        pick_points = {'0': [], '45': [], '90': [], '135': []}
        for point in points:
            angle = np.arctan2(point[1] - min_z_point[1], point[0] - min_z_point[0])
            angle_degrees = np.degrees(angle)

            if np.abs(angle_degrees) <= 3 or np.abs(angle_degrees) >= 177:
                pick_points['0'].append(point)

            elif 87 <= np.abs(angle_degrees) <= 93:
                pick_points['90'].append(point)

            elif (42 <= np.abs(angle_degrees) <= 48) or (132 <= np.abs(angle_degrees) <= 138):
                pick_points['45' if point[0] * point[1] >= 0 else '135'].append(point)

        angles = []
        steepness = []
        for angle, points_list in pick_points.items():
            if len(points_list) > 0:
                sampled_points = sample_points(np.array(points_list), num_samples=1000)
                angles.append(angle)
                steepness.append(calc_z_diff_over_xy(sampled_points, min_z_point))

        return angles[np.argmax(steepness)]

    @staticmethod
    def calc_quantiles_angle(mask):
        """
        硬质玩具的抓取点和角度
        """
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        rect = cv2.minAreaRect(contours[0])
        center_rect = rect[0]  # w * h
        rect_coor = np.array(cv2.boxPoints(rect))

        angle = -(90 - int(rect[2])) if rect[1][0] > rect[1][1] else int(rect[2])  # if w > h

        distances = []
        for i in range(4):
            for j in range(i + 1, 4):
                dist = np.linalg.norm(rect_coor[i] - rect_coor[j])
                distances.append((i, j, dist))

        distances.sort(key=lambda x: x[2])
        short_side1_midpoint = np.array(((rect_coor[distances[0][0]][0] + rect_coor[distances[0][1]][0]) / 2,
                                         (rect_coor[distances[0][0]][1] + rect_coor[distances[0][1]][1]) / 2))
        short_side2_midpoint = np.array(((rect_coor[distances[1][0]][0] + rect_coor[distances[1][1]][0]) / 2,
                                         (rect_coor[distances[1][0]][1] + rect_coor[distances[1][1]][1]) / 2))

        short_size1_endpoints = np.array([rect_coor[distances[0][0]], rect_coor[distances[0][1]]])
        one_quarter_quantile = (short_side1_midpoint + center_rect) / 2
        three_quarter_quantile = (short_side2_midpoint + center_rect) / 2

        return [center_rect, one_quarter_quantile, three_quarter_quantile], short_size1_endpoints, angle

    def pixel_to_3d(self, depth, pixel_u, pixel_v):
        fx = self.camera_matrix[0][0]
        fy = self.camera_matrix[1][1]
        ppx = self.camera_matrix[0][2]
        ppy = self.camera_matrix[1][2]

        d = depth[int(pixel_v), int(pixel_u)] / 1000.0
        x = (pixel_u - ppx) * d / fx
        y = (pixel_v - ppy) * d / fy

        return np.array([x, y, d])
