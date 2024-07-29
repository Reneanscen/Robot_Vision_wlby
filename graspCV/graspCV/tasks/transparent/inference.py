import os
import cv2
import numpy as np
from tasks.transparent.corelibs.opts import opts
from scipy.spatial.transform import Rotation
from tasks.transparent.utils.triangulation import Triangulation
from tasks.transparent.corelibs.models.lightglue import LGHomography
from tasks.transparent.corelibs.detectors.detector_factory import detector_factory


class TransParentModel:

    def __init__(self, model_arch: str, center_pose_weight_path: str, lg_homography_weight_path):
        self.center_bb_model_arch = model_arch
        self.center_pose_weight_path = center_pose_weight_path
        self.lg_homography_weight_path = lg_homography_weight_path
        self.center_pose, self.light_glue = self.get_model()

    def get_model(self):
        # load model
        opt = opts().parser.parse_args()
        opt.use_pnp = True
        opt.nms = True
        opt.obj_scale = True
        opt.task = 'object_pose'
        opt.use_pnp = True
        opt.arch = self.center_bb_model_arch
        opt.load_model = self.center_pose_weight_path

        opt = opts().parse(opt)
        opt = opts().init(opt)
        os.environ['CUDA_VISIBLE_DEVICES'] = opt.gpus_str
        opt.debug = max(opt.debug, 1)
        Detector = detector_factory[opt.task]
        detector = Detector(opt)

        lgh = LGHomography(self.lg_homography_weight_path)
        return detector, lgh

    def get_two_image_matched_kps(self, ref_image, image, ref_bbox, bbox):
        kps1, kps2 = self.light_glue(ref_image, image, ref_bbox, bbox)
        return kps1, kps2

    def get_detection_dict(self, image, cam_intrinsic):
        bboxes, bboxes_projected_points, rotations_w2c_list, locations, \
            bboxes_cls = self._get_detection_data(image, cam_intrinsic)

        det_res = {
            "bboxes": bboxes,
            "bboxes_projected_points": bboxes_projected_points,
            "rotations_w2c_list": rotations_w2c_list,
            "locations": locations,
            "bboxes_cls": bboxes_cls
        }

        return det_res

    def _get_detection_data(self, image, cam_intrinsic):

        meta = {"camera_matrix": cam_intrinsic}
        ret_results = self.center_pose.run(image, meta_inp=meta)
        bboxes = list()
        bboxes_cls = list()  # maybe we can do classification to bbox
        bboxes_projected_points = list()
        rotations_w2c_list = list()
        locations = list()
        for det in ret_results['results']:
            if 'kps_3d_cam' not in det.keys() or 'quaternion_xyzw' not in det.keys() or 'bbox' not in det.keys():
                continue

            bboxes.append(det['bbox'])
            # todo add classification model

            bbox3d = det['kps_3d_cam']
            bbox_projected_points = list()
            for i in range(bbox3d.shape[0]):
                left_uvz = cam_intrinsic @ bbox3d[i].T
                bbox_projected_points.append(left_uvz[0:2] / left_uvz[2:])
            bboxes_projected_points.append(bbox_projected_points)

            quaternion = det['quaternion_xyzw'][0]
            location = np.asarray(det['location'])
            rotation = Rotation.from_quat(quaternion).as_matrix()
            rotations_w2c_list.append(rotation)
            locations.append(location)

        return bboxes, bboxes_projected_points, rotations_w2c_list, locations, bboxes_cls


def get_iou(bbox1, bbox2):
    x_min1, y_min1, x_max1, y_max1 = bbox1[0], bbox1[1], bbox1[2], bbox1[3]
    x_min2, y_min2, x_max2, y_max2 = bbox2[0], bbox2[1], bbox2[2], bbox2[3]
    x_intersection = max(0, min(x_max1, x_max2) - max(x_min1, x_min2))
    y_intersection = max(0, min(y_max1, y_max2) - max(y_min1, y_min2))
    intersection = x_intersection * y_intersection
    area_bbox1 = (x_max1 - x_min1) * (y_max1 - y_min1)
    area_bbox2 = (x_max2 - x_min2) * (y_max2 - y_min2)
    union = area_bbox1 + area_bbox2 - intersection
    iou = intersection / union
    return iou


def match_bboxes_by_key_points(boxes1, boxes2, kps1, kps2):
    box_num1 = len(boxes1)
    box_num2 = len(boxes2)
    match_mat = np.zeros(shape=(box_num1, box_num2), dtype=np.float32)

    for kp1, kp2 in zip(kps1, kps2):
        for idx1, box1 in enumerate(boxes1):
            for idx2, box2 in enumerate(boxes2):
                l1 = box1[0] / 0.9
                t1 = box1[1] / 0.9
                r1 = box1[2] / 1.1
                b1 = box1[3] / 1.1

                l2 = box2[0] / 0.9
                t2 = box2[1] / 0.9
                r2 = box2[2] / 1.1
                b2 = box2[3] / 1.1

                if l1 < kp1[0] < r1 and t1 < kp1[1] < b1 and l2 < kp2[0] < r2 and t2 < kp2[1] < b2:
                    match_mat[idx1, idx2] += 1

    match_mat[match_mat < 3] = 0
    vis_bbox(bbox1=boxes1, bbox2=boxes2)

    row_match = list()
    for i in range(box_num1):
        min_score = 0
        tmp_match = -1
        for j in range(box_num2):
            if min_score < match_mat[i, j] and (j not in row_match):
                tmp_match = j
        row_match.append(tmp_match)

    return row_match


def triangulate(pts: list,
                T: np.ndarray,
                K: np.ndarray):
    """

    :param pts: [[x1, y1],...[xn, yn]]  matched n-points(from n frame)
    :param T: [[[R, t], [0, 1]]]        each camera pose matrix represent a world to i-th camera transform(rotation and translation), n-1 of 4X4 ndarray
    :param K: [3X3 intrinsic matrix]    camera intrinsic
    :return:
    """

    cams_name = list()
    cams_matrix = dict()
    for idx in range(len(pts)):
        cams_matrix["cam_{}".format(idx)] = K
        cams_name.append("cam_{}".format(idx))

    triangulation = Triangulation(camera_matrix=cams_matrix)
    # vis_points(pts)
    for idx in range(len(pts)):
        triangulation.add_projection(cams_name[idx], T[idx], pts[idx])

    xyz = triangulation.triangulate()
    errors = triangulation.compute_error(xyz)
    error = sum(errors) / len(errors)

    return xyz, error



def vis_bbox(bbox1, bbox2, width=1280, height=720):
    img_black = np.zeros(shape=(height, width, 3), dtype=np.uint8)
    bbox1 = np.asarray(bbox1, dtype=np.int32)
    bbox2 = np.asarray(bbox2, dtype=np.int32)

    for idx, box in enumerate(bbox1):
        x1, y1, x2, y2 = box[0], box[1], box[2], box[3]
        cv2.putText(img_black, str(idx), org=(int(x1) - 5, int(y1) - 5), fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                    fontScale=0.5, color=(25, 120, 78))
        cv2.rectangle(img_black, pt1=(x1, y1), pt2=(x2, y2), color=(255, 0, 0), thickness=2)

    for idx, box in enumerate(bbox2):
        x1, y1, x2, y2 = box[0], box[1], box[2], box[3]
        cv2.putText(img_black, str(idx), org=(int(x1) - 5, int(y1) - 5), fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                    fontScale=0.5, color=(25, 120, 78))
        cv2.rectangle(img_black, pt1=(x1, y1), pt2=(x2, y2), color=(0, 255, 0), thickness=2)

    cv2.imshow("eval bbox", img_black)


def vis_points(points):
    image = np.zeros(shape=(720, 1280, 3), dtype=np.uint8)
    for p in points:
        cv2.circle(image, (int(p[0]), int(p[1])), radius=3, thickness=2, color=(0, 255, 0))
    cv2.imshow("point vis", image)
    cv2.waitKey(1)
