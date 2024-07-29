import cv2
import numpy as np
from tasks.transparent.corelibs.models.onnx_runner import (
    SuperPointLightGlueEnd2EndRunner,
    resize_image,
    rgb_to_grayscale,
    normalize_image
)


class LGHomography:

    def __init__(self, wight_path="weights/superpoint_lightglue_end2end.onnx"):
        self.scale_max = 512
        self.runner = SuperPointLightGlueEnd2EndRunner(
            onnx_path=wight_path,
            providers=["CPUExecutionProvider"],
        )

    def __call__(self, image1, image2, bboxes1, bboxes2, debug=True):
        kps1, kps2 = self.kp_match(image1, image2)

        if debug:
            image1_copy = cv2.copyTo(image1, mask=None)
            image2_copy = cv2.copyTo(image1, mask=None)
            for p in kps1:
                cv2.circle(image1_copy, (int(p[0]), int(p[1])), radius=2, color=(0, 255, 0), thickness=2)

            for p in kps2:
                cv2.circle(image2_copy, (int(p[0]), int(p[1])), radius=2, color=(0, 255, 0), thickness=2)

            cv2.imshow("kp_match1", image1_copy)
            cv2.imshow("kp_match2", image2_copy)
            cv2.waitKey(30)

        return kps1, kps2

    def kp_match(self, image1, image2):

        image1, scales1 = resize_image(image1, self.scale_max, fn='max')
        image2, scales2 = resize_image(image2, self.scale_max, fn='max')

        image1 = normalize_image(image1)[None].astype(np.float32)
        image2 = normalize_image(image2)[None].astype(np.float32)
        image1 = rgb_to_grayscale(image1)
        image2 = rgb_to_grayscale(image2)
        kp1, kp2 = self.runner.run(image1, image2, scales1, scales2)

        return kp1, kp2
