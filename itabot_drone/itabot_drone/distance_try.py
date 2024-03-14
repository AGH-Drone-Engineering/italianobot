import cv2 
import numpy as np

calib_data_path = "/home/ember/italianobot/itabot_drone/calib_data/MultiMatrix.npz"
calib_data = np.load(calib_data_path)
cam_mat = calib_data["camMatrix"]
dist_coef = calib_data["distCoef"]
r_vectors = calib_data["rVector"]
t_vectors = calib_data["tVector"]


image_points = cv2.imread('/home/ember/italianobot/itabot_drone/itabot_drone/images/image1.png')
print(image_points.shape)
points = [(320, 180)]
points = np.array(points, dtype=np.float32)
undistorted_points = cv2.undistortPoints(points, cam_mat, dist_coef)

print(undistorted_points[0][0][0])
# print(undistorted_points)