import os
import sys
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from robot_cv_msg.msg import RgbD
from robot_cv_msg.srv import GraspSrv
from robot_cv_msg.msg import GraspInfo
from robot_cv_msg.msg import Matrix4x4

sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from core import TaskManager, logger    # noqa
from core.download_models import check_model    # noqa
from configs import MODEL_PREFIX    # noqa


class GraspCVServer(Node):
    def __init__(self, name="grasp_cv_node"):
        super(GraspCVServer, self).__init__(name)

        self.declare_parameter("max_actives", 3)
        self.declare_parameter("service_name", "grasp_cv_service")
        self.declare_parameter("realsense_topic_name", "/realsense/image_raw")
        self.declare_parameter("kinect_topic_name", "/kinect/image_raw")

        max_actives = self.get_parameter("max_actives").get_parameter_value().integer_value
        service_name = self.get_parameter("service_name").get_parameter_value().string_value
        realsense_topic_name = self.get_parameter("realsense_topic_name").get_parameter_value().string_value
        kinect_topic_name = self.get_parameter("kinect_topic_name").get_parameter_value().string_value

        # check/download models
        check_model(prefix=MODEL_PREFIX)

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1)
        self.server = self.create_service(GraspSrv, service_name, self.service_callback)
        # subscribe camera
        self.realsense_node = Node("realsense")
        self.kinect_node = Node("kinect")
        self.realsense_sub = self.realsense_node.create_subscription(RgbD, realsense_topic_name, self.image_callback, qos_profile)
        self.kinect_sub = self.kinect_node.create_subscription(RgbD, kinect_topic_name, self.image_callback, qos_profile)

        self.task_manager = TaskManager(max_actives)
        self.image = None
        self.depth = None
        self.max_try_cnt = 10

        logger.info("Service started")

    def image_callback(self, msg: RgbD):
        self.image = np.frombuffer(msg.rgb.data, dtype=np.uint8).reshape((msg.rgb.height, msg.rgb.width, 3))
        self.depth = np.frombuffer(msg.depth.data, dtype=np.uint16).reshape((msg.rgb.height, msg.rgb.width))

    def service_callback(self, request: GraspSrv.Request, response: GraspSrv.Response): # noqa
        # parse task code
        logger.info("step into callback")
        robot_extrinsic = np.array(request.robot_rt.elements, dtype=np.float32).reshape([4, 4])
        task_code = request.task_code
        task_id = task_code // 1000
        target_id = task_code % 1000
        clear = request.clear

        # get image
        try_cnt = 0
        image, depth = None, None
        node = self.kinect_node if (task_id == 0 and target_id == 0) else self.realsense_node
        while try_cnt < self.max_try_cnt:
            logger.info("getting images")
            rclpy.spin_once(node)
            logger.info("spin once")
            if self.image is None:
                try_cnt += 1
            else:
                image = self.image
                depth = self.depth
                self.image = None
                self.depth = None
                break

        if try_cnt >= self.max_try_cnt:
            logger.error("get image error")
            response.state = False
            return response
        logger.info("get images")
        # perform task
        task = self.task_manager.get_task(task_id)
        results = task(image,
                       depth=depth,
                       task_id=task_id,
                       target_id=target_id,
                       robot_extrinsic=robot_extrinsic,
                       clear=clear)

        # response
        if isinstance(results, bool) and results:
            response.state = True
        elif isinstance(results, list):
            if len(results) == 0:
                response.state = False
            else:
                response.state = True
                for result in results:
                    grasp_info = GraspInfo()
                    grasp_info.target_id = int(f"{result['target_id']:03d}")
                    grasp_info.grasp_scores = result["grasp_scores"]
                    grasp_info.loc_type = result["loc_type"]
                    for rt in result["grasp_rt"]:
                        grasp_rt = Matrix4x4()
                        grasp_rt.elements = rt.reshape(-1).tolist()
                        grasp_info.grasp_matrices.append(grasp_rt)
                    response.grasp_rt.append(grasp_info)
        else:
            response.state = False
        logger.info("service call finished")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = GraspCVServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
