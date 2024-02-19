import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cv2
from cv2 import aruco
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped


class ArucoDetector(Node):
    def __init__(self):
        super().__init__("aruco_detector")
        self.bridge = CvBridge()
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.image_sub = self.create_subscription(
            CompressedImage,
            "/camera/color/image_raw/compressed",
            self.callback,
            qos_profile,
        )

        # Creating aruco distance from rosbot publisher
        # X [->] right direction on camera
        # Y [V] down direction on camera
        # Z distance

        self.distance_pub = self.create_publisher(String, "/distance", 10)
        self.position_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/aruco/pose", 10
        )

        # Camera calibration:
        self.calib_data_path = "../calib_data/MultiMatrix.npz"
        self.calib_data = np.load(self.calib_data_path)
        self.cam_mat = self.calib_data["camMatrix"]
        self.dist_coef = self.calib_data["distCoef"]
        self.r_vectors = self.calib_data["rVector"]
        self.t_vectors = self.calib_data["tVector"]
        self.MARKER_SIZE = 10  # centimeters
        self.marker_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
        self.param_markers = aruco.DetectorParameters_create()

    def callback(self, msg):
        try:
            # Decode the compressed image
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            marker_corners, marker_IDs, reject = aruco.detectMarkers(
                gray_frame, self.marker_dict, parameters=self.param_markers
            )
            if marker_corners:
                rVec, tVec, _ = aruco.estimatePoseSingleMarkers(
                    marker_corners, self.MARKER_SIZE, self.cam_mat, self.dist_coef
                )
                total_markers = range(0, marker_IDs.size)
                for ids, corners, i in zip(marker_IDs, marker_corners, total_markers):
                    cv2.polylines(
                        frame,
                        [corners.astype(np.int32)],
                        True,
                        (0, 255, 255),
                        4,
                        cv2.LINE_AA,
                    )
                    corners = corners.reshape(4, 2)
                    corners = corners.astype(int)
                    top_right = corners[0].ravel()
                    top_left = corners[1].ravel()
                    bottom_right = corners[2].ravel()
                    bottom_left = corners[3].ravel()

                    distance = np.sqrt(
                        tVec[i][0][2] ** 2 + tVec[i][0][0] ** 2 + tVec[i][0][1] ** 2
                    )
                    point = cv2.drawFrameAxes(
                        frame, self.cam_mat, self.dist_coef, rVec[i], tVec[i], 4, 4
                    )
                    cv2.putText(
                        frame,
                        f"id: {ids[0]} Dist: {round(distance, 2)}",
                        tuple(top_right),
                        cv2.FONT_HERSHEY_PLAIN,
                        1.3,
                        (0, 0, 255),
                        2,
                        cv2.LINE_AA,
                    )
                    cv2.putText(
                        frame,
                        f"x:{round(tVec[i][0][0],1)} y: {round(tVec[i][0][1],1)} ",
                        tuple(bottom_right),
                        cv2.FONT_HERSHEY_PLAIN,
                        1.0,
                        (0, 0, 255),
                        2,
                        cv2.LINE_AA,
                    )
                    try:
                        message = String()
                        message.data = f"id: {ids[0]} Dist: {round(distance, 2)} x:{round(tVec[i][0][0],1)} y: {round(tVec[i][0][1],1)}"
                        self.distance_pub.publish(message)

                        initpose = PoseWithCovarianceStamped()
                        initpose.header.stamp = Clock().now()
                        initpose.header.frame_id = "map"

                        initpose.pose.pose.position.x = distance
                        initpose.pose.pose.position.y = -round(tVec[i][0][0], 1)
                        initpose.pose.pose.positon.z = -round(tVec[i][0][1], 1)
                        initpose.pose.pose.orientation.w = 1
                        initpose.pose.pose.orientation.x = 0
                        initpose.pose.pose.orientation.y = 0
                        initpose.pose.pose.orientation.z = 0
                        self.position_pub.publish(initpose)

                    except:
                        self.get_logger().info("Publisher error")

            cv2.imshow("frame", frame)
            key = cv2.waitKey(1)
            if key == ord("q"):
                cv2.destroyAllWindows()
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