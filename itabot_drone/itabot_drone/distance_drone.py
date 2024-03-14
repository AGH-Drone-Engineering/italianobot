import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import cv2
import cv2.aruco as aruco
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
import tf_transformations as tf_trans
import tf2_ros
import os
from tf_transformations import quaternion_multiply
from drone_controller import DroneControl
import time

def pixel(src):
    gray = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)

    gray = cv2.medianBlur(gray, 7)
    
    _, mask = cv2.threshold(gray, 250, 255, cv2.THRESH_BINARY)

    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5,5), np.uint8))

    src[mask > 0] = (0, 0, 255)

    cnts, _ = cv2.findContours(mask, 1, 2)
    x = 0.0
    y = 0.0
    if len(cnts) > 0:
        best_size = -1
        for i in range(len(cnts)):
            cnt = cnts[i]
            M = cv2.moments(cnt)
            cx = M['m10'] / M['m00']
            cy = M['m01'] / M['m00']
            size = M['m00']
            if size >= best_size:
                x = cx
                y = cy
                best_size = size



    return np.array([x, y])

class ArucoDetector(Node):

    def __init__(self):

        super().__init__("aruco_detector")
        self.bridge = CvBridge()

        # Camera subscribers:

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        print("XDDD")

        self.image_sub = self.create_subscription(
            Image,
            "/camera/drone",
            self.img_callback,
            qos_profile,
        )
        self.dron = DroneControl()
        self.dron.takeoff()
        self.calib_data_path = "/home/ember/italianobot/itabot_drone/calib_data/MultiMatrix.npz"
        self.calib_data = np.load(self.calib_data_path)
        self.cam_mat = self.calib_data["camMatrix"]
        self.dist_coef = self.calib_data["distCoef"]
        self.r_vectors = self.calib_data["rVector"]
        self.t_vectors = self.calib_data["tVector"]
        time.sleep(6)
        
        """ self.cam_mat = np.zeros((3, 3))
        self.dist_coef = np.zeros(5)
        """


    def img_callback(self, msg):
        try:
            large_code_position = [0,0,0]
            
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            
            self.points = pixel(frame)
            
            undistorted_points = cv2.undistortPoints(self.points, self.cam_mat, self.dist_coef)
            self.dron.face_north()
            self.dron.move(x=-undistorted_points[0][0][0], y=-undistorted_points[0][0][1], z=0)
            print(undistorted_points)
            

            cv2.line(frame, (int(self.points[0]), 0), (int(self.points[0]), frame.shape[1]), (0, 0, 255))
            cv2.line(frame, (0, int(self.points[1])), (frame.shape[0], int(self.points[1])), (0, 0, 255))
            # cv2.imshow("frame", frame)
            # key = cv2.waitKey(1)
            # if key == ord("q"):
            #     cv2.destroyAllWindows()

        except CvBridgeError as e:
            self.get_logger().error("Could not convert image: %s" % e)


def main(args=None):
    rclpy.init(args=args)
    aruco_detector = ArucoDetector()
    rclpy.spin(aruco_detector)
    aruco_detector.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
