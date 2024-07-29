import os
import math
import numpy as np
from numpy import ndarray
from core import TaskInterface
from core.logger import logger
from multiprocessing import Queue
from typing import Union, Tuple, List
from scipy.spatial.transform import Rotation
from configs import REAL_SENSE_D435i_CAM_INTRINSIC, TARGET_MAP, MODEL_SAVE_DIR, MODEL_PREFIX, ROBOT_HAND_BASE_FOR_CUP
from tasks.transparent.inference import TransParentModel, triangulate, match_bboxes_by_key_points


class TransGrasp(TaskInterface):

    def __init__(self,
                 model_arch: str,
                 center_pose_model_path: str,
                 lg_homography_model_path: str,
                 input_shape: Union[Tuple, List] = (1280, 720),  # [height, width]
                 reproj_err_thresh: float = 300,
                 return_big_reproj_err: bool = True,
                 generate_degrees: float = 15.0,
                 backend: str = "pytorch",
                 debug: bool = False, **kwargs):
        super(TransGrasp, self).__init__("", input_shape, backend, debug)

        self.tpm = None
        self.model_arch = model_arch
        self.center_pose_weight_path = os.path.join(MODEL_SAVE_DIR, MODEL_PREFIX, center_pose_model_path)
        self.lg_homography_weight_path = os.path.join(MODEL_SAVE_DIR, MODEL_PREFIX, lg_homography_model_path)
        self._load_model()
        self.reproj_err_thresh = reproj_err_thresh
        self.return_big_reproj_err = return_big_reproj_err
        self.img_q = Queue()
        self.det_q = Queue()
        self.rt_q = Queue()
        self.generate_degrees = generate_degrees
        self.cam_matrix = np.asarray(REAL_SENSE_D435i_CAM_INTRINSIC, dtype=np.float32)

    def _infer_pytorch(self, image: ndarray, **kwargs):
        det_result = self.tpm.get_detection_dict(image, self.cam_matrix)
        return det_result

    def _preprocess(self, image: ndarray) -> ndarray:
        return image

    def _post_process(self, outputs, **kwargs) -> Union[List, bool]:
        image = kwargs.get("image", None)
        clear = kwargs.get("clear", None)
        robot_extrinsic = kwargs.get("robot_extrinsic", None)
        self.img_q.put(image)
        self.det_q.put(outputs)
        self.rt_q.put(robot_extrinsic)

        if clear:
            pose_result = list()
            image_list = list()
            det_dict_list = list()
            rt_list = list()
            # convert queue to list
            while self.img_q.qsize() > 0 and self.det_q.qsize() > 0 and self.rt_q.qsize() > 0:
                det_dict = self.det_q.get()
                if len(det_dict["bboxes"]) > 0:
                    image_list.append(self.img_q.get())
                    det_dict_list.append(det_dict)
                    rt_list.append(self.rt_q.get())
            det_success_times = len(image_list)
            if det_success_times < 2:
                return pose_result

            ref_idx = 0
            transforms = list()
            proj_points_list = list()
            ref_bboxes_proj_points = det_dict_list[ref_idx]["bboxes_projected_points"]
            proj_points_list.append(ref_bboxes_proj_points)
            transforms.append(rt_list[ref_idx])
            for idx_i in range(1, det_success_times):

                kps1, kps2 = self.tpm.get_two_image_matched_kps(image_list[ref_idx],
                                                                image_list[idx_i],
                                                                det_dict_list[ref_idx]["bboxes"],
                                                                det_dict_list[idx_i]["bboxes"])

                row_match = match_bboxes_by_key_points(det_dict_list[ref_idx]["bboxes"],
                                                       det_dict_list[idx_i]["bboxes"],
                                                       kps1, kps2)

                transforms.append(rt_list[idx_i])
                proj_points_matched = list()
                for idx in row_match:
                    if idx != -1:
                        proj_points_matched.append(det_dict_list[idx_i]["bboxes_projected_points"][idx])
                    else:
                        proj_points_matched.append(None)
                proj_points_list.append(proj_points_matched)

            for box_idx in range(len(ref_bboxes_proj_points)):  # correspond to reference frame detected bbox idx
                matched_points = list()
                matched_T = list()
                for frame_idx in range(len(proj_points_list)):  # correspond to frame idx
                    if proj_points_list[frame_idx][box_idx] is not None:
                        matched_points.append(proj_points_list[frame_idx][box_idx][0])
                        matched_T.append(transforms[frame_idx])

                if len(matched_T) >= 2:
                    logger.info("one box matched cup nums {}".format(len(matched_T)))
                    xyz, err = triangulate(matched_points, np.asarray(matched_T), REAL_SENSE_D435i_CAM_INTRINSIC)  # get 3d xyz
                    if err < self.reproj_err_thresh:
                        logger.info("reproject error: {}".format(err))
                    else:
                        if not self.return_big_reproj_err:
                            logger.critical("reproject error: {}, maybe wrong triangulation".format(err))
                            continue
                        logger.warning("reproject error: {}, maybe wrong triangulation".format(err))
                    if xyz is not None:
                        logger.info("orig position: {}".format(xyz))
                        logger.info("biased position: {}".format(xyz))
                        # obj to cam rotation dot to obj to world coordinate
                        # ( in this case, world coordinate is manipulator base coordinate)
                        r_obj2world = np.asarray([[-9.39438e-05, 6.42964e-05, 1],
                                                  [0.707105, 0.707109, 2.09636e-05],
                                                  [-0.707109, 0.707105, - 0.000111893]], dtype=np.float32)

                        pose_list = self.get_rotation_for_obj(r_obj2world, xyz, degrees=self.generate_degrees)

                        res = {
                            "grasp_scores": 1.0,
                            "grasp_rt": pose_list,
                            "target_id": TARGET_MAP["110"]["0"],
                            "loc_type": 1
                        }
                        pose_result.append(res)

            return pose_result

        else:
            if len(outputs["bboxes"]) > 0:
                return True
            else:
                return False

    def _load_model_pytorch(self):
        self.tpm = TransParentModel(self.model_arch, self.center_pose_weight_path, self.lg_homography_weight_path)

    @staticmethod
    def get_rotation_for_obj(base_rt, xyz, degrees=15):
        xy = xyz[0:2]
        fake_x = 1
        fake_y = xy[0] * fake_x / xy[1]
        start_theta = math.atan(fake_x / fake_y)
        radians = math.radians(degrees)
        # generate pose
        count = 1
        pose_list = list()
        while True:
            current_radians = start_theta + count * radians
            if start_theta + math.pi < current_radians:
                pose_rotate_euler = [current_radians, 0, 0]
                R = Rotation.from_euler("xyz", pose_rotate_euler, degrees=False)    # noqa
                R_3x3 = R.as_matrix()       # noqa
                base_pose = np.eye(4)        # noqa
                pose = np.dot(R_3x3, base_rt)
                base_pose[0: 3, 3] += xyz
                base_pose[0: 3, 0: 3] += pose
                pose_list.append(base_pose)
                count += 1
            else:
                break

        return pose_list
