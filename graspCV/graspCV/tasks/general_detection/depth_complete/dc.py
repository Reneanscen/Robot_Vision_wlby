import os
import cv2
import torch
import numpy as np
from time import perf_counter
from tasks.general_detection.depth_complete.models.Tode import Tode


class DepthComplete(object):

    def __init__(self, configs):

        self.device = "cuda:0" #if torch.cuda.is_available() and configs['use_cuda'] else "cpu"

        self.model = Tode.build(**configs['params'])
        if os.path.isfile(configs['weight_path']):
            state_dict = torch.load(configs['weight_path'], map_location=self.device)
            self.model.load_state_dict(state_dict['model_state_dict'])
        self.model.to(self.device)

        self.image_size = configs['image_size']
        self.depth_min = configs['depth_min']
        self.depth_max = configs['depth_max']
        self.depth_norm = configs['depth_norm']
        self.target_size = configs['target_size']

    def __call__(self, rgb, depth):
        return self.inference(rgb, depth, self.target_size)

    def inference(self, rgb, depth, target_size=(1280, 720)):
        """
        Inference.

        Parameters
        ----------

        rgb, depth: the initial RGB-D image;

        target_size: tuple of (int, int), optional, default: (1280, 720), the target depth image size.

        Returns
        -------

        The depth image after completion.
        """

        rgb = cv2.resize(rgb, self.image_size, interpolation=cv2.INTER_LINEAR)
        depth = cv2.resize(depth, self.image_size, interpolation=cv2.INTER_NEAREST)
        rgb_float = np.asarray(rgb, dtype=np.float32)
        depth_float = np.asarray(depth, dtype=np.float32)
        depth_float = depth_float / self.depth_norm
        depth_float = np.where(depth_float < self.depth_min, 0, depth_float)
        depth_float = np.where(depth_float > self.depth_max, 0, depth_float)
        depth_float[np.isnan(depth_float)] = 0
        rgb_float = (rgb_float / 255.0).transpose(2, 0, 1)
        rgb_float = torch.FloatTensor(rgb_float).to(self.device).unsqueeze(0)
        depth_float = torch.FloatTensor(depth_float).to(self.device).unsqueeze(0)
        with torch.no_grad():
            time_start = perf_counter()
            depth_res = self.model(rgb_float, depth_float)
            time_end = perf_counter()

        print("Inference finished, time: {:.4f}s.".format(time_end - time_start))
        depth_res = depth_res.squeeze(0).squeeze(0).cpu().detach().numpy()
        depth_res = depth_res * self.depth_norm
        depth_res = cv2.resize(depth_res, target_size, interpolation=cv2.INTER_NEAREST)
        return depth_res
