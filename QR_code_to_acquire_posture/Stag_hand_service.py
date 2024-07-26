import rclpy
from rclpy.node import Node
from robot_cv_msg.srv import Pose
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField, Image, CameraInfo
from custom_image_msg.msg import Image4m, String
import cv2
import array
import numpy as np
import time
from geometry_msgs.msg import Transform, Pose
from geometry_msgs.msg import Quaternion
import stag
from cv_bridge import CvBridge



class PoseService(Node):

    def __init__(self):
        super().__init__('pose_service')

        self.bridge = CvBridge()
        # self.srv = self.create_service(Pose, 'get_pose', self.handle_get_pose)

        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=3)

        # 创建 RGB 和深度图像的订阅对象
        self.rgb_subscription = self.create_subscription(Image4m, '/hand_cam_color', self.get_rgb_image, qos_profile=self.qos_profile)

        self.publisher = self.create_publisher(Pose, '/object_detection', 10)


        self.rgb_image_data = None

        # SDK
        # self.mtx = np.array([[435.06, 0, 323.367],
        #                      [0, 433.885, 243.434],
        #                      [0, 0, 1]], dtype=np.float32)

        # dedistortion
        self.mtx = np.array([[396.01322774,   0., 318.7676843 ],
                            [0., 395.84024203, 239.50347013],
                            [0.,   0.,   1.]], dtype=np.float32)

        # nodistortion
        # self.mtx = np.array([[395.83039261, 0., 321.89194502],
        #                      [0., 395.76372835, 243.27620742],
        #                      [0., 0., 1.]], dtype=np.float32)

        # SDK
        # self.dist = np.array([-0.0543285, 0.0599714, 0.00029117, 0.00170801, -0.0199568], dtype=np.float32)

        # dedistortion
        self.dist = np.array([-0.0065515 ,  0.01131358, -0.00043684,  0.000167  , -0.00158916], dtype=np.float32)

        # nodistortion
        # self.dist = np.array([-0.06347068,  0.10239164, -0.00024269,  0.00118628, -0.07312021], dtype=np.float32)

        self.w = 2
        self.h = 2

        self.worldpoint = self.generate_world_points()
        self.world = np.array([[0., 0., 0.],
                           [50., 0., 0.],
                           [50., 50., 0.],
                           [0., 50., 0.]])

        self.counter = 0

        self.end_to_camera = np.array([[0.00486563, 0.99882776, -0.04816046, -0.08082581],
                                       [-0.99998435, 0.004993, 0.0025249, 0.00770852],
                                       [0.00276241, 0.04814742, 0.99883642, -0.16919281],
                                       [0, 0, 0, 1]])

    def generate_world_points(self):
        objp = np.zeros((self.w * self.h, 3), np.float32)
        grid = np.mgrid[0:self.w, 0:self.h].T.reshape(-1, 2)
        grid_sorted = np.zeros_like(grid)
        grid_sorted[:, 0] = grid[:, 0]
        grid_sorted[:, 1] = grid[:, 1]
        grid_sorted[2, :], grid_sorted[3, :] = grid[3, :], grid[2, :]

        # 将重排后的坐标赋值给 objp
        objp[:, :2] = grid_sorted
        worldpoint = objp * 50  # 棋盘格的宽度为15mm (array(70,3))
        return worldpoint

    def get_rgb_image(self, msg):
        # 获取 RGB 图像数据

        # rgb_recv_time = time.time()
        # print("Receive rgb cost time:", rgb_recv_time - msg.header.stamp.sec - msg.header.stamp.nanosec / 1e9)

        help_image_msg = Image()
        help_image_msg.step = msg.step
        length = msg.height * msg.step
        help_image_msg.data = array.array('B')
        help_image_msg.data.frombytes(msg.data[:length].tobytes())
        help_image_msg.encoding = 'bgr8'
        help_image_msg.height = msg.height
        help_image_msg.width = msg.width
        help_image_msg.is_bigendian = msg.is_bigendian


        cv_image = self.bridge.imgmsg_to_cv2(help_image_msg, desired_encoding='passthrough')
        self.rgb_image_data = cv_image

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        (corners, ids, rejected_corners) = stag.detectMarkers(gray, 15)
        if len(ids) >= 1:

            if 0 < corners[0][0][0][0] < 640 and \
                0 < corners[0][0][1][0] < 640 and \
                0 < corners[0][0][2][0] < 640 and \
                0 < corners[0][0][3][0] < 640 and \
                0 < corners[0][0][0][1] < 480 and \
                0 < corners[0][0][1][1] < 480 and \
                0 < corners[0][0][2][1] < 480 and \
                0 < corners[0][0][3][1] < 480:

                corners2 = cv2.cornerSubPix(gray, corners[0], (5, 5), (-1, -1),
                                            (cv2.TERM_CRITERIA_MAX_ITER | cv2.TERM_CRITERIA_EPS, 30, 0.001))

                if [corners2]:
                    corners = corners2

            _, rvec, tvec = cv2.solvePnP(self.worldpoint, corners[0], self.mtx, self.dist)  # 解算位姿

            # Convert rvec to rotation matrix
            rotation_matrix, _ = cv2.Rodrigues(rvec)

            transformation_matrix = np.eye(4)

            # 将旋转矩阵放入变换矩阵的左上角
            transformation_matrix[:3, :3] = rotation_matrix

            # 将平移向量放入变换矩阵的最后一列
            transformation_matrix[:3, 3] = tvec.flatten()
            transformation_matrix[0, 3] /= 1000
            transformation_matrix[1, 3] /= 1000
            transformation_matrix[2, 3] /= 1000


            final_transformation_matrix = np.dot(self.end_to_camera, transformation_matrix)

            pose = Pose()
            # 平移部分
            pose.position.x = final_transformation_matrix[0, 3]
            pose.position.y = final_transformation_matrix[1, 3]
            pose.position.z = final_transformation_matrix[2, 3]

            # 旋转部分 (转换为四元数)
            rot = transformation_matrix[:3, :3]
            quat = self.rotation_matrix_to_quaternion(rot)
            pose.orientation.x = quat[0]
            pose.orientation.y = quat[1]
            pose.orientation.z = quat[2]
            pose.orientation.w = quat[3]

            self.publisher.publish(pose)

        # self.filename = f"calib/image_{self.counter}.png"
        # self.counter += 1

        # rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

        # cv2.imwrite(self.filename, cv_image)

    # def handle_get_pose(self, request, response):
    #     command = request.command
    #     # 根据接收到的指令，生成pose矩阵
    #     (corners, ids, rejected_corners) = stag.detectMarkers(self.rgb_image_data, 15)
    #     if len(ids) >= 1:
    #
    #         _, rvec, tvec = cv2.solvePnP(self.worldpoint, corners[0], self.mtx, self.dist)  # 解算位姿
    #
    #         # Convert rvec to rotation matrix
    #         rotation_matrix, _ = cv2.Rodrigues(rvec)
    #
    #         # Create Transform message
    #         transform = Transform()
    #         transform.translation.x = float(tvec[0])
    #         transform.translation.y = float(tvec[1])
    #         transform.translation.z = float(tvec[2])
    #
    #         # Convert rotation matrix to quaternion
    #         transform.rotation = self.rotation_matrix_to_quaternion(rotation_matrix)
    #
    #         response.transform = transform
    #     return response

    # def rotation_matrix_to_quaternion(self, R):
    #     # Ensure R is a 3x3 matrix
    #     assert R.shape == (3, 3)
    #
    #     # Calculate the trace of the matrix
    #     trace = np.trace(R)
    #     if trace > 0:
    #         s = 0.5 / np.sqrt(trace + 1.0)
    #         qw = 0.25 / s
    #         qx = (R[2, 1] - R[1, 2]) * s
    #         qy = (R[0, 2] - R[2, 0]) * s
    #         qz = (R[1, 0] - R[0, 1]) * s
    #     else:
    #         if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
    #             s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
    #             qw = (R[2, 1] - R[1, 2]) / s
    #             qx = 0.25 * s
    #             qy = (R[0, 1] + R[1, 0]) / s
    #             qz = (R[0, 2] + R[2, 0]) / s
    #         elif R[1, 1] > R[2, 2]:
    #             s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
    #             qw = (R[0, 2] - R[2, 0]) / s
    #             qx = (R[0, 1] + R[1, 0]) / s
    #             qy = 0.25 * s
    #             qz = (R[1, 2] + R[2, 1]) / s
    #         else:
    #             s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
    #             qw = (R[1, 0] - R[0, 1]) / s
    #             qx = (R[0, 2] + R[2, 0]) / s
    #             qy = (R[1, 2] + R[2, 1]) / s
    #             qz = 0.25 * s
    #
    #     return Quaternion(x=qx, y=qy, z=qz, w=qw)
    def rotation_matrix_to_quaternion(self, R):
        q = np.empty((4,))
        t = np.trace(R)
        if t > 0.0:
            t = np.sqrt(t + 1.0)
            q[3] = 0.5 * t
            t = 0.5 / t
            q[0] = (R[2, 1] - R[1, 2]) * t
            q[1] = (R[0, 2] - R[2, 0]) * t
            q[2] = (R[1, 0] - R[0, 1]) * t
        else:
            i = 0
            if R[1, 1] > R[0, 0]:
                i = 1
            if R[2, 2] > R[i, i]:
                i = 2
            j = (i + 1) % 3
            k = (j + 1) % 3
            t = np.sqrt(R[i, i] - R[j, j] - R[k, k] + 1.0)
            q[i] = 0.5 * t
            t = 0.5 / t
            q[3] = (R[k, j] - R[j, k]) * t
            q[j] = (R[j, i] + R[i, j]) * t
            q[k] = (R[k, i] + R[i, k]) * t
        return q


def main(args=None):
    rclpy.init(args=args)
    pose_service = PoseService()
    rclpy.spin(pose_service)
    pose_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()