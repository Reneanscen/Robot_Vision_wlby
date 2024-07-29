import os


__all__ = ["TASK_MAP", "TARGET_MAP", "FINE_LOC", "TRIPLE_LOC", "ONCE_LOC",
           "REAL_SENSE_D435i_CAM_INTRINSIC", "K4A_CAM_INTRINSIC", "COLOR_PLATE",
           "MODEL_SAVE_DIR", "MODEL_PREFIX", "FINE_LOC_GET_MULTI_POSE",
           "ROBOT_HAND_BASE_FOR_CUP", "ROBOT_HAND_BASE_FOR_STORAGE", "OBJ_BIAS"]

# REAL_SENSE_D435i_CAM_INTRINSIC = [[907.99633789, 0., 641.96173096],
#                                   [0., 907.30743408, 381.2472229],
#                                   [0., 0., 1., ]]
# K4A_CAM_INTRINSIC = [[907.99633789, 0., 641.96173096],
#                      [0., 907.30743408, 381.2472229],
#                      [0., 0., 1., ]]
REAL_SENSE_D435i_CAM_INTRINSIC = [[912.01690674, 0., 653.86376953],
                                  [0., 912.33929443, 372.13464355],
                                  [0., 0., 1.]]
K4A_CAM_INTRINSIC = [[613.04321289, 0., 640.05224609],
                     [0., 613.09606934, 367.93109131],
                     [0., 0., 1.]]

# ROBOT_HAND_BASE_FOR_CUP = [[-9.71664e-05, 0.000115429, 1, 0.616319],
#                            [0.76603, 0.642805, 2.33833e-07, 1.4626e-07],
#                            [-0.642805, 0.76603, -0.000150881, 0.736295],
#                            [0, 0, 0, 1]]

ROBOT_HAND_BASE_FOR_CUP = [[1, 0, 0, 0],
                           [0, 1, 0, 0],
                           [0, 0, 1, 0],
                           [0, 0, 0, 1]]

ROBOT_HAND_BASE_FOR_STORAGE = [[-0.004587, -0.00685797, -0.999966, 0.45],
                               [0.0192572, -0.999792, 0.00676844, 0],
                               [-0.999804, -0.0192255, 0.00471811, 0],
                               [0, 0, 0, 1]]

OBJ_BIAS = {"cup": 0.05, }

# Note: key defined for model path must be "model_path" or like "xxx_model_path";
# otherwise, model can not be downloaded.
TASK_MAP = {
    # task id: [task, {args1, arg2, arg3, ...}]
    220: ["GraspShoes", {"model_path": "shoes_detect_with_rotation_s.onnx",
                         "input_shape": (640, 640),
                         "num_classes": 4,
                         "backend": "onnxruntime",
                         "debug": False}],
    000: ["GeneralDetection", {"detect_model_path": "house_obj_0921.pt",
                               "depth_refine_model_path": "transcg_checkpoint.tar",
                               "input_shape": (640, 640),
                               "backend": "pytorch",
                               "debug": True,
                               "rotate_degree": 30,
                               "mask_filter_thresh": 100}],
    210: ["GraspStorage", {"model_path": "house_obj_0921.pt",
                           "input_shape": (640, 640),
                           "backend": "pytorch",
                           "debug": False}],
    110: ["TransGrasp", {"model_arch": "dla_34",
                         "center_pose_model_path": "cup_cup_v1_sym_12_140.pth",
                         "lg_homography_model_path": "superpoint_lightglue_end2end.onnx",
                         "input_shape": (1280, 720),
                         "reproj_err_thresh": 300,
                         "return_big_reproj_err": True,
                         "generate_degrees": 15.0,
                         "backend": "pytorch"}]
}

"""
task_id:
    generic_det: 000000
    cup: 110000
    storage_things: 210000
    shoes: 220000
"""

FINE_LOC = ["push_pedal", "cup"]
TRIPLE_LOC = ["transparent_cup"]
ONCE_LOC = ["shoe_left", "shoe_right", "clothes", "plastic_toy", "plush_toy"]
FINE_LOC_GET_MULTI_POSE = ["cup"]

TARGET_MAP = {
    # task_id: {cls_id_in_model: target_id, ...}
    220: {0: 10, 1: 11, 2: 16, 3: 17},
    000: {
        "transparent_mug": 1,
        "mug": 2,
        "cup": 3,
        "transparent_bottle": 4,
        "transparent_cup": 5,
        "transparent_salt_shaker": 6,
        "bottle": 7,
        "push_pedal": 8,
        "water_dispenser": 9,
        "shoe_left": 10,
        "shoe_right": 11,
        "clothes": 12,
        "plastic_toy": 13,
        "plush_toy": 14,
        "door_handle": 15,
        "slipper_left": 16,
        "slipper_right": 17,
        "snacks": 18,
    },
    210: {11: 11, 12: 12, 13: 13},
    110: {0: 5}
}


COLOR_PLATE = [(76, 47, 71), (88, 85, 242), (119, 143, 253), (99, 209, 202), (66, 206, 81),
               (54, 41, 158), (49, 208, 144), (100, 2, 127), (188, 138, 139), (41, 186, 73),
               (48, 234, 43), (13, 176, 129), (55, 184, 29), (164, 131, 132), (217, 72, 173),
               (34, 55, 184), (128, 164, 131), (173, 217, 72)]

MODEL_PREFIX = "wlby_models"
MODEL_SAVE_DIR = os.path.join(os.path.expanduser("~"), ".cache/graspCV/models")
