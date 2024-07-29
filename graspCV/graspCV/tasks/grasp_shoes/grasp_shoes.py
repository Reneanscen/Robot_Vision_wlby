import cv2
import torch
import numpy as np
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


class GraspShoes(TaskInterface):    # noqa
    def __init__(self, model_path: str,
                 input_shape: Union[Tuple, List] = (640, 640),  # [height, width]
                 num_classes: int = None,
                 backend: str = "tensorrt",
                 target_map: Dict = None,
                 debug: bool = False,
                 score_thresh: float = 0.5,
                 iou_thresh: float = 0.45,
                 stride: int = 32):
        super(GraspShoes, self).__init__(model_path, input_shape, num_classes, backend, target_map, debug)

        self.score_thresh = score_thresh
        self.iou_thresh = iou_thresh
        self.stride = stride

        # intrinsic matrix: realsense D435i 1280x720
        # focal = [909.62, 908.809, 643.526, 362.554]    # [fx, fy, u0, v0]
        # self.camera_matrix = np.array([[focal[0], 0,        focal[2]],
        #                                [0,        focal[1], focal[3]],
        #                                [0,        0,        1]], dtype=np.float32)
        self.camera_matrix_inv = np.linalg.inv(REAL_SENSE_D435i_CAM_INTRINSIC)

    def _preprocess(self, image: ndarray) -> ndarray:
        if self.backend == "pytorch":
            return image

        img = LetterBox(self.input_shape, auto=False, stride=self.stride)(image=image)
        img = np.expand_dims(img, axis=0)
        img = img[..., ::-1].transpose((0, 3, 1, 2))  # BGR to RGB, BHWC to BCHW, (n, 3, h, w)
        img = np.ascontiguousarray(img).astype(np.float32)  # contiguous
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
        R_matrix = kwargs.get("robot_extrinsic")    # noqa

        if self.backend == "pytorch":
            bboxes = outputs.boxes.xyxy.cpu().numpy()       # [N, 4]
            if bboxes.shape[0] == 0:    # don't detect shoes
                return []
            scores = outputs.boxes.conf.cpu().numpy()       # [N]
            classes = outputs.boxes.cls.cpu().numpy()       # [N]
            rotations = outputs.rotations.cpu().numpy()     # [N, 2]
            masks = scale_image(outputs.masks.data.permute(1, 2, 0).cpu().numpy(), (ih, iw))
            rotations = np.arctan2(rotations[:, 0], rotations[:, 1])
        else:
            outputs = [torch.from_numpy(out) for out in outputs]
            p = non_max_suppression(outputs[0],
                                    self.score_thresh,
                                    self.iou_thresh,
                                    agnostic=False,
                                    max_det=300,
                                    nc=self.num_classes)

            pred = p[0]     # remove batch
            if pred.shape[0] == 0:
                logger.info("detect 0 shoes")
                return []

            bboxes = pred[:, :4]
            scores = pred[:, 4].numpy()
            classes = pred[:, 5].numpy().astype(int)
            rotations = pred[:, 6:8]
            mask_preds = pred[:, 8:]

            masks = process_mask(outputs[1][0], mask_preds, bboxes, self.input_shape, upsample=True)
            masks = scale_image(masks.permute(1, 2, 0).numpy(), (ih, iw))
            bboxes = scale_boxes(self.input_shape, bboxes, (ih, iw)).numpy()
            rotations = np.arctan2(rotations[:, 0].numpy(), rotations[:, 1].numpy())

        results = []
        logger.info(f"detect {len(bboxes)} shoes")
        for i, bbox in enumerate(bboxes):
            mask = masks[:, :, i]   # [ih, iw]
            score = scores[i]       # [1]
            cls = classes[i]        # [1]
            rot = rotations[i]      # [1]

            # filter objects according to target_id
            flag = False
            if target_id == 0:
                flag = True
            elif target_id == 10 and cls in [0, 1]:
                flag = True
            elif target_id == 20 and cls in [2, 3]:
                flag = True
            elif target_id in [11, 12, 21, 22] and cls == target_id:
                flag = True
            if not flag:
                continue

            is_slipper = cls in [2, 3]

            grasp_rot = rot + np.pi / 2     # zme
            # grasp_rot = rot + np.pi / 6
            # if is_slipper:
            #     if rot >= 0:
            #         grasp_rot = rot - np.pi / 2
            #     else:
            #         grasp_rot = rot + np.pi / 2
            # else:
            #     if np.pi / 2 < rot <= np.pi:
            #         grasp_rot = rot - np.pi
            #     elif -np.pi <= rot < -np.pi / 2:
            #         grasp_rot = rot + np.pi
            #     else:
            #         grasp_rot = rot

            # compute depth of grasp point
            d = self.compute_grasp_point_depth(depth, mask, offset=-0.03 if is_slipper else 0.04)

            # compute u/v coordinate of grasp point (must use the detected radians)
            u, v, contours = self.compute_grasp_point_uv(np.where(mask != 0, 1, 0).astype(np.uint8),
                                                         rot,
                                                         offset=0.1 if is_slipper else 0.2,
                                                         is_slipper=is_slipper)

            grasp_point_3d = np.dot(self.camera_matrix_inv, np.array([u, v, 1]).reshape((3, 1)) * d)  # [3, 1] (x, y, z)
            logger.info(f"camera xyz: {grasp_point_3d}")
            grasp_point_4d = np.ones((4, 1), dtype=np.float32)  # [4, 1] (x, y, z, 1)
            grasp_point_4d[:3] = grasp_point_3d
            logger.info(f"rotation angle: {grasp_rot / np.pi * 180}")
            if is_slipper:
                # TODO: how to compute Rt
                r1 = Quaternion(axis=[0, 1, 0], degrees=-90).rotation_matrix
                r2 = Quaternion(axis=[1, 0, 0], degrees=90).rotation_matrix
                r3 = Quaternion(axis=[0, 0, 1], degrees=-grasp_rot).rotation_matrix
                r = np.dot(r3, np.dot(r2, r1))
            else:
                r = Quaternion(axis=[0, 0, 1], radians=-grasp_rot).rotation_matrix  # [3, 3]

            rt = np.eye(4, dtype=np.float32)
            rt[:3, :3] = r
            rt[:, 3:4] = grasp_point_4d
            rt = np.dot(R_matrix, rt)
            logger.info(f"camera rt: {r}")
            logger.info(f"output rt: {rt}")

            result = {"target_id": self.target_map[cls], "grasp_scores": [float(score)], "grasp_rt": [rt], "loc_type": 1}
            results.append(result)

            if self.debug:
                rect = cv2.minAreaRect(contours)
                box = cv2.boxPoints(rect).astype(int)
                center = rect[0]

                cv2.drawContours(image, contours, -1, (0, 0, 255), 2)
                dx = np.cos(rot) * 100
                dy = np.sin(rot) * 100
                end_point = (int(center[0] + dx), int(center[1] - dy))
                cv2.arrowedLine(image, (int(center[0]), int(center[1])), end_point, (0, 255, 0), 1)
                cv2.circle(image, (int(u), int(v)), 6, (255, 0, 0), -1)
                cv2.putText(image, f"{score:.3f} {cls}", tuple(bbox[:2].astype(int)), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                            (0, 0, 255), 2)

                cv2.line(image, tuple(box[0]), tuple(box[1]), (0, 0, 255), 2)
                cv2.line(image, tuple(box[1]), tuple(box[2]), (0, 0, 255), 2)
                cv2.line(image, tuple(box[2]), tuple(box[3]), (0, 0, 255), 2)
                cv2.line(image, tuple(box[3]), tuple(box[0]), (0, 0, 255), 2)

        if self.debug:
            cv2.imshow("image", image)
            cv2.waitKey(1)

        return results

    @staticmethod
    def compute_grasp_point_uv(mask, radians, offset=0.2, is_slipper=False):
        """
            ______*______
            |           |
            |           |
            |           |
            *     *     *
            |           |
            |           |
            |_____*_____|

            *: centerx
        """
        assert 0 <= offset <= 1
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contour_id = 0
        if len(contours) > 1:   # to filter small mask
            max_area = -1
            for i in range(len(contours)):
                contour_area = cv2.contourArea(contours[i])
                if contour_area > max_area:
                    max_area = contour_area
                    contour_id = i
        rect = cv2.minAreaRect(contours[contour_id])     # ((x_center, y_center), (w, h), angle)
        box = cv2.boxPoints(rect)               # [4, 2]

        center = rect[0]
        center0 = (box[0] + box[1]) // 2
        center1 = (box[1] + box[2]) // 2
        center2 = (box[2] + box[3]) // 2
        center3 = (box[3] + box[0]) // 2
        angle0 = np.arctan2(center[1] - center0[1], center0[0] - center[0])
        angle1 = np.arctan2(center[1] - center1[1], center1[0] - center[0])
        angle2 = np.arctan2(center[1] - center2[1], center2[0] - center[0])
        angle3 = np.arctan2(center[1] - center3[1], center3[0] - center[0])

        angles = np.array([angle0, angle1, angle2, angle3])
        diff_angles = np.abs(np.abs(angles - radians) - (np.pi if not is_slipper else 0))
        idx = np.argmin(diff_angles)

        choose_center = [center0, center1, center2, center3][idx]
        choose_angle = [angle0, angle1, angle2, angle3][idx]
        half_len = np.sqrt(np.square(center - choose_center).sum())
        choose_len = half_len * (offset if is_slipper else 1 - offset)
        dx = choose_len * np.cos(choose_angle)
        dy = choose_len * np.sin(choose_angle)

        u, v = center[0] + dx, center[1] - dy

        return u, v, contours[contour_id]

    @staticmethod
    def compute_grasp_point_depth(depth, mask, offset=0.004):
        depths = depth[mask != 0].reshape(-1)
        depths.sort()
        d_len = len(depths)
        depths = depths[int(d_len * 0.1): int(d_len * 0.9)]
        d = np.mean(depths) / 1000.0 + offset
        return d

