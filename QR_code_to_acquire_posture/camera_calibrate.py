import numpy as np
import glob
import cv2


objp = np.zeros((11 * 8, 3), np.float32)
objp[:, :2] = np.mgrid[0:11, 0:8].T.reshape(-1, 2)  
objp = 3 * objp 
obj_points = [] 
img_points = [] 
images = glob.glob("/home/zme/ultralytics/data/realsense/data_0524/images/*.png")  

print(images)

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    size = gray.shape[::-1]
    ret, corners = cv2.findChessboardCorners(gray, (11, 8), None)
    if ret:
        obj_points.append(objp)
        corners2 = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1),
                                    (cv2.TERM_CRITERIA_MAX_ITER | cv2.TERM_CRITERIA_EPS, 30, 0.001))
        if [corners2]:
            img_points.append(corners2)
        else:
            img_points.append(corners)
        cv2.drawChessboardCorners(img, (11, 8), corners, ret)  
        cv2.waitKey(1)
_, mtx, dist, _, _ = cv2.calibrateCamera(obj_points, img_points, size, None, None)

Camera_intrinsic = {"mtx": mtx, "dist": dist, }

print(Camera_intrinsic)
