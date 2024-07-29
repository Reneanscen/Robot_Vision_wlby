import cv2
import torch
from ultralytics import YOLO


class YoloDetection:

    def __init__(self, weight_path: str, use_cuda=True):

        self.yolo_mask_model = YOLO(weight_path, task='segment')
        self.device = "cuda:0" if torch.cuda.is_available() and use_cuda else "cpu"
        self.yolo_mask_model.to(self.device)

    def __call__(self, image):
        return self.forward(image)

    def forward(self, image):
        ret_dict = dict()

        print("进入yolo 前向")
        param = {
            'conf': 0.3,
            'save': False,
            'imgsz': 1280
        }
        h, w = image.shape[0: 2]
        results = self.yolo_mask_model(image, **param)
        print("yolo 前向结束")
        boxes_info = results[0].boxes
        masks = results[0].masks
        name_dict = results[0].names
        detected_class = boxes_info.cls.data.cpu().numpy().tolist()

        print("检测到 {} 个目标".format(len(detected_class)))
        for idx, cls_idx in enumerate(detected_class):
            if name_dict[cls_idx] not in ret_dict.keys():
                ret_dict[name_dict[cls_idx]] = list()
            mask_and_box = dict()
            mask = masks[idx].data.cpu().numpy().squeeze()
            mask = cv2.resize(mask, (w, h), cv2.INTER_NEAREST)
            mask_and_box["mask"] = mask
            mask_and_box["box"] = boxes_info[idx].xyxy.cpu().numpy().squeeze()
            mask_and_box["conf"] = boxes_info[idx].conf.data.cpu().numpy().squeeze()
            ret_dict[name_dict[cls_idx]].append(mask_and_box)

        return ret_dict





