import cv2
import numpy as np
from scipy.spatial.transform import Rotation


def skew(v):
    x = v[0][0]
    y = v[1][0]
    z = v[2][0]
    m = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
    return m


class Triangulation:
    def __init__(self, camera_matrix):
        # calibration matrix
        self._camera_matrix = camera_matrix
        self.projections = {}
        for (camera_name, _) in camera_matrix.items():
            self.projections[camera_name] = []

        self.A = None  # Ax = b
        self.b = None

    def add_projection(self, camera_name, pose_camera_world, projection):
        self.projections[camera_name].append((pose_camera_world, projection))

    def compute_error(self, Pw):
        errors = []
        for (camera_name, projections) in self.projections.items():
            for (pose_camera_world, projection) in projections:
                K = self._camera_matrix[camera_name]
                R = pose_camera_world[0:3, 0:3]
                t = pose_camera_world[0:3, 3:4]
                Pc = np.matmul(R, Pw) + t
                Pc[0][0] /= Pc[2][0]
                Pc[1][0] /= Pc[2][0]
                Pc[2][0] = 1
                Puv = np.matmul(K, Pc)
                Euv = Puv - np.array([[projection[0]], [projection[1]], [1]])
                error = np.linalg.norm(Euv)
                errors.append(error)
        return errors

    def triangulate(self):
        number_projections = 0
        for (camera_name, projections) in self.projections.items():
            number_projections += len(projections)

        if number_projections < 2:
            return np.array([[0], [0], [0]])

        # assign memory
        self.A = np.zeros((number_projections * 2, 3))
        self.b = np.zeros((number_projections * 2, 1))

        index = 0
        for (camera_name, projections) in self.projections.items():
            for (pose_camera_world, projection) in projections:
                A, b = self._compute_equation(camera_name, pose_camera_world, projection)
                self.A[index:index + 2, :] = A[0:2, :]
                self.b[index:index + 2, :] = b[0:2, :]
                index += 2
        At = np.transpose(self.A)
        AtA = np.matmul(At, self.A)
        Atb = np.matmul(At, self.b)
        return np.matmul(np.linalg.inv(AtA), Atb)

    def _compute_equation(self, camera_name, pose_camera_world, projection):
        K = self._camera_matrix[camera_name]
        R = pose_camera_world[0:3, 0:3]
        t = pose_camera_world[0:3, 3:4]
        Puv = np.array([[projection[0]], [projection[1]], [1]])
        Pc = np.matmul(np.linalg.inv(K), Puv)
        Pcx = skew(Pc)
        A = np.matmul(-Pcx, R)
        b = np.matmul(Pcx, t)
        return A, b


def IterationInsection(pts, K, T):

    k0 = 1.5
    k1 = 2.5  # K1=2
    weight = np.identity(len(pts) * 2)
    cam_xyz = mutiTriangle(pts, K, T)
    cam_xyz_pre = cam_xyz
    iteration = 0
    while 1:
        d = np.zeros((len(pts), 1))
        for i in range(len(T)):
            R_t = T[i]
            R = R_t[0: 3, 0: 3]
            t = R_t[:3, 3]
            R_vector = Rotation.from_matrix(R).as_euler("XYZ")
            pro, J = cv2.projectPoints(cam_xyz.reshape(1, 1, 3), R_vector, t, K, np.array([]))
            pro = pro.reshape(1, 2)
            deltax = pro[0, 0] - pts[i][0, 0]
            deltay = pro[0, 1] - pts[i][0, 1]
            d[i, 0] = np.sqrt(deltax ** 2 + deltay ** 2)
        weight_temp = np.diag(weight)[::2].reshape(-1, 1)
        delta = np.sqrt(np.sum(weight_temp * d ** 2) / (len(pts) - 2))
        w = np.zeros((len(pts), 1))
        for i in range(len(pts)):
            u = d[i]
            if abs(u) < k0 * delta:
                w[i] = 1
            elif abs(u) < k1 * delta and abs(u) >= k0 * delta:
                w[i] = delta / u
            elif abs(u) >= k1 * delta:
                w[i] = 0
        weight_temp = w
        weight_p = [val for val in weight_temp.reshape(-1, ) for i in range(2)]
        weight_p = np.diag(weight_p)
        cam_xyz_curr = weight_mutiTriangle(pts, K, T, weight_p)
        dx = cam_xyz_curr[0, 0] - cam_xyz_pre[0, 0]
        dy = cam_xyz_curr[1, 0] - cam_xyz_pre[1, 0]
        dz = cam_xyz_curr[2, 0] - cam_xyz_pre[2, 0]
        # print(dx,dy,dz)
        if np.sqrt(dx ** 2 + dy ** 2 + dz ** 2) < 0.01:
            break
        else:
            cam_xyz = cam_xyz_curr
            cam_xyz_pre = cam_xyz_curr
            weight = weight_p
            iteration += 1
    #    print("d{0}".format(d))
    print("iteration is {0}\n".format(iteration))
    print("IGG....{0},{1},{2}".format(cam_xyz[0, 0], cam_xyz[1, 0], cam_xyz[2, 0]))
    return cam_xyz, weight


def weight_mutiTriangle(pts, K, T, weight):
    if len(pts) >= 3:
        equa_A = []
        equa_b = []
        for i in range(len(pts)):
            R_t = T[i]
            R = R_t[0: 3, 0: 3]
            t = R_t[:3, 3]
            camPts = pixelToCam(pts[i], K)
            t1 = np.dot(np.linalg.inv(R), - t).reshape(1, 3)
            A1, b1 = getEquation(camPts, R[i], t1)
            equa_A.append(A1)
            equa_b.append(b1)
        AA = np.vstack(equa_A)
        bb = np.vstack(equa_b)
        P_ls = np.dot(np.linalg.pinv(AA.T @ weight @ AA), AA.T @ weight @ bb)
        return P_ls
    else:
        print("tracker pixel point less 4,can not insection........")
        return None


def mutiTriangle(pts, K, T):
    if len(pts) >= 3:  # 这里是假设至少track 4帧
        equa_A = []
        equa_b = []
        for i in range(len(pts)):
            R_t = T[i]
            R = R_t[0: 3, 0: 3]
            t = R_t[:3, 3]
            camPts = pixelToCam(pts[i], K)
            t1 = np.dot(np.linalg.inv(R), -t).reshape(1, 3)
            A1, b1 = getEquation(camPts, R, t1)
            equa_A.append(A1)
            equa_b.append(b1)
        AA = np.vstack(equa_A)
        bb = np.vstack(equa_b)
        P_ls = np.dot(np.linalg.inv(AA.T @ AA), AA.T @ bb)
        return P_ls
    else:
        print("tracker pixel point less 3,can not insection........")
        return None


def pixelToCam(pts, K):
    '''

    :param pts: pixel coordinates
    :param K: camera params
    :return: camera coordinates
    '''
    camPts = np.zeros((1, 2))
    camPts[0, 0] = (pts[0] - K[0, 2]) / K[0, 0]
    camPts[0, 1] = (pts[1] - K[1, 2]) / K[1, 1]
    return camPts


def getEquation(camPts, R, t):
    '''
    build equation ,one pixel point get 2 equations
    :param camPts: camera coordinates
    :param R: image pose-rotation ,world to camera
    :param t: image pose -translation,is camera center(t=-R.T*tvec)
    :return: equation coefficient
    '''
    A = np.zeros((2, 3))
    b = np.zeros((2, 1))
    A[0, 0] = R[0, 0] - camPts[0, 0] * R[2, 0]
    A[0, 1] = R[0, 1] - camPts[0, 0] * R[2, 1]
    A[0, 2] = R[0, 2] - camPts[0, 0] * R[2, 2]
    b[0, 0] = t[0, 0] * A[0, 0] + t[0, 1] * A[0, 1] + t[0, 2] * A[0, 2]
    A[1, 0] = R[1, 0] - camPts[0, 1] * R[2, 0]
    A[1, 1] = R[1, 1] - camPts[0, 1] * R[2, 1]
    A[1, 2] = R[1, 2] - camPts[0, 1] * R[2, 2]
    b[1, 0] = t[0, 0] * A[1, 0] + t[0, 1] * A[1, 1] + t[0, 2] * A[1, 2]
    return A, b
