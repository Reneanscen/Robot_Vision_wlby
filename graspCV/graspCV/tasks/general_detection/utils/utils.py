import cv2
import numpy as np
from scipy.spatial.transform import Rotation
import scipy.signal as signals
from core.logger import logger
from configs import COLOR_PLATE


def get_location(image: np.ndarray,
                 mask: np.ndarray,
                 depth: np.ndarray,
                 cam_intrinsic: np.ndarray,
                 mask_area_thresh=100,
                 in_meter=True):
    pose = np.zeros(shape=[4, 4], dtype=np.float32)

    mask = np.asarray(mask, dtype=np.uint8)
    depth = np.asarray(depth, dtype=np.float32)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    max_contour = contours[0]
    max_area = 0.0
    for cont in contours:
        area = cv2.contourArea(cont)
        if max_area < area:
            max_contour = cont
            max_area = area
    if max_area < mask_area_thresh:
        return pose, False, image

    center_point = cv2.minAreaRect(max_contour)[0]
    box = cv2.boundingRect(max_contour)

    x1, y1 = box[0], box[1]
    x2, y2 = box[2] + x1, box[3] + y1
    mask = np.zeros_like(depth, dtype=np.uint8)
    cv2.circle(mask, center=(int(center_point[0]), int(center_point[1])),
               radius=3, color=(1,), thickness=cv2.FILLED)

    if len(depth.shape) == 2:
        depth_roi = depth[y1: y2, x1: x2]
        mask_roi = mask[y1: y2, x1: x2]
    elif len(depth.shape) == 3 and depth.shape[2] == 1:
        depth_roi = depth[y1: y2, x1: x2, 0]
        mask_roi = mask[y1: y2, x1: x2, 0]
    else:
        return None
    kernel = cv2.getStructuringElement(shape=cv2.MORPH_ELLIPSE, ksize=(3, 3))
    depth_roi = signals.medfilt2d(depth_roi, kernel_size=(3, 3))
    depth_roi = cv2.dilate(depth_roi, kernel)
    depth_roi = cv2.erode(depth_roi, kernel)
    mask_roi = cv2.dilate(mask_roi, kernel)

    if in_meter:
        depth_roi /= 1000
    mask_roi[depth_roi < 0.3] = 0
    mask_roi[depth_roi > 1.5] = 0
    depth_roi = depth_roi * mask_roi + (1 - mask_roi) * 10000
    x = center_point[0]
    y = center_point[1]
    depth_mean = cv2.mean(depth_roi, mask_roi)[0]
    logger.debug(depth_mean)
    cv2.circle(image, center=(int(x), int(y)), color=(0, 255, 0), radius=3, thickness=2)

    xyz = np.asarray([x, y, 1], dtype=np.float32) * depth_mean
    xyz = np.linalg.inv(cam_intrinsic).dot(xyz)
    return xyz


def get_coarse_or_fine_location(image: np.ndarray,
                                mask: np.ndarray,
                                depth: np.ndarray,
                                cam_intrinsic: np.ndarray,
                                cam_to_base: np.ndarray,
                                mask_area_thresh=100,
                                fine=True,
                                coarse_loc_time=3,
                                rotate_degree=30,
                                robot_hand_base=None,
                                in_meter=True):

    xyz_in_cam = get_location(image, mask,  depth, cam_intrinsic, mask_area_thresh=mask_area_thresh, in_meter=in_meter)

    if not isinstance(xyz_in_cam, np.ndarray):
        logger.warning("Can't pre location by depth map ( maybe bad measurement? )")
        return None, False, image

    logger.info(f"XYZ in camera: {xyz_in_cam}")
    base_pose = np.eye(4, dtype=np.float32)
    if fine:
        pose = np.eye(4, dtype=np.float32)
        pose[0: 3, 3] = xyz_in_cam
        pose = cam_to_base.dot(pose)
        if isinstance(robot_hand_base, np.ndarray):
            pose[0: 3, 0: 3] = robot_hand_base[0: 3, 0: 3]
        logger.info(f"one location pose : {pose}")
        return pose, True, image
    elif coarse_loc_time == 1:
        xyz_in_robot = cam_to_base.dot(np.append(xyz_in_cam, 1))
        pose = np.eye(4, dtype=np.float32)
        if isinstance(robot_hand_base, np.ndarray):
            pose = robot_hand_base
        pose[0: 3, 3] += xyz_in_robot[0:3]
        logger.info(f"one location pose : {pose}")
        return pose, True, image

    elif coarse_loc_time == 3:
        min_loc = (coarse_loc_time - 1) * rotate_degree // 2 - (coarse_loc_time - 1) * rotate_degree + 60
        pose_list = list()
        base_pose[2, 3] = -0.20

        for i in range(coarse_loc_time):
            pose_rotate_euler = [0, min_loc + i * 30, 0]
            R = Rotation.from_euler("xyz", pose_rotate_euler, degrees=True)
            R_3x3 = R.as_matrix()
            T_4x4 = np.eye(4)
            T_4x4[0: 3, 0: 3] = R_3x3
            pose_in_obj = T_4x4.dot(base_pose)
            pose_in_obj[0: 3, 3] += np.asarray(xyz_in_cam)
            pose_in_obj = cam_to_base.dot(pose_in_obj)
            logger.info(f"{i}th position: {pose_in_obj}")
            pose_list.append(pose_in_obj)
        pose_list = list(reversed(pose_list))
        return pose_list, True, image

    else:
        logger.critical("No such definition")
        return None, False, image


def vis_object(image, result_dict):
    image = np.asarray(image, dtype=np.float32)
    for idx, cls_name in enumerate(result_dict.keys()):
        det_results = result_dict[cls_name]
        for det in det_results:
            mask = det["mask"]
            bbox = det["box"]
            confidence = float(det["conf"])
            cv2.rectangle(image, pt1=(int(bbox[0]), int(bbox[1])), pt2=(int(bbox[2]), int(bbox[3])),
                          color=COLOR_PLATE[idx], thickness=2)
            cv2.putText(image, org=(int(bbox[0]), int(bbox[1]) - 15),
                        text=cls_name + ":" + str(round(confidence, 2)),
                        thickness=2, fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.5, color=COLOR_PLATE[idx])

            mask = np.expand_dims(mask, axis=-1)
            image = image * mask * 0.55 + mask * COLOR_PLATE[idx] * 0.45 + image * (1 - mask)
    image = np.asarray(image, dtype=np.uint8)
    return image
