import numpy as np
import pyrealsense2 as rs


class RealsenseCamera(object):
    """
    A camera module that provides color and depth stream.
    Camera intrinsics accessible with self.cam_k.
    """

    RESOLUTION = {
        480: (640, 480),
        720: (1280, 720),
        1080: (1920, 1080),
    }

    def __init__(self, resulution=720, fps=30, device_id=""):
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        self.device_id = device_id
        self.config.enable_device(device_id)
        # if self.device_id:
        #     ctx = rs.context()
        #     for device in ctx.query_devices():
        #         device_id = device.get_info(rs.camera_info.serial_number)
        #         if device_id == self.device_id:
        #             self.config.enable_device(device_id)

        # Get device product line for setting a supporting resolution
        pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        pipeline_profile = self.config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()
        device_product_line = str(device.get_info(rs.camera_info.product_line))
        if self.device_id:
            device_serial_number = str(device.get_info(rs.camera_info.serial_number))
            print(device_serial_number)

        found_rgb = False
        for s in device.sensors:
            if s.get_info(rs.camera_info.name) in ['RGB Camera', 'Stereo Module']:
                found_rgb = True
                break
        if not found_rgb:
            print("The demo requires Depth camera with Color sensor")
            exit(0)

        # self.config.enable_stream(rs.stream.depth, *self.RESOLUTION[resulution], rs.format.z16, fps)
        self.config.enable_stream(rs.stream.color, *self.RESOLUTION[resulution], rs.format.bgr8, fps)

        # Start streaming
        profile = self.pipeline.start(self.config)

        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            raise RuntimeError(f"realsense get error!")

    def get_image(self):
        # Get frameset of color
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        # Validate that color frame are valid
        if not color_frame:
            return None

        color_image = np.asanyarray(color_frame.get_data())

        return color_image

    def __del__(self):
        self.pipeline.stop()


if __name__ == '__main__':
    import cv2
    import time

    camera = RealsenseCamera(resulution=480, fps=60, device_id="233522076056")
    # camera = RealsenseCamera(resulution=480, fps=60, device_id="323622270006")
    t0 = time.time()
    cnt = 0
    while True:
        frame = camera.get_image()

        cv2.imshow("realsense view", frame)
        t = time.time()
        cnt += 1
        print(f"fps: {cnt / (t - t0):.2f}")
        if cv2.waitKey(1) == 27:
            break
