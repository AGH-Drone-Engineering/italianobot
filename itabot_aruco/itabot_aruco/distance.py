import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import cv2
from cv2 import aruco
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf_transformations as tf_trans


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

        self.image_sub = self.create_subscription(
            CompressedImage,
            "/camera/color/image_raw/compressed",
            self.img_callback,
            qos_profile,
        )

        self.calib_sub = self.create_subscription(
            CameraInfo,
            "/camera/color/camera_info",
            self.calib_callback,
            qos_profile,
        )

        # Aruco position publishers:
        self.aruco_publishers = dict()

        self.distance_pub = self.create_publisher(String, "/distance", 10)
        # do testow awaryjnie (potem do usuniecia)

        # Aruco code parameters:

        self.ARUCO_CODE_SIZE_CM = 10  # centimeters
        self.MARKER_SIZE = (
            self.ARUCO_CODE_SIZE_CM / 100
        )  # tVec unit is meters so conversion is necessary

        self.marker_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
        self.param_markers = aruco.DetectorParameters_create()

        # Camera calibration from file:

        """ # self.calib_data_path = "../calib_data/MultiMatrix.npz"
        self.calib_data_path = (
            "/home/husarion/ros2_ws/src/itabot_aruco/calib_data/MultiMatrix.npz"
        )
        self.calib_data = np.load(self.calib_data_path)
        self.cam_mat = self.calib_data["camMatrix"]
        self.dist_coef = self.calib_data["distCoef"]
        # unnecessary:
        # self.r_vectors = self.calib_data["rVector"]
        # self.t_vectors = self.calib_data["tVector"] """

    def calib_callback(self, msg):
        # Camera calibration from topic:

        self.cam_mat = np.array(msg.k).reshape((3, 3))
        self.dist_coef = np.array(msg.d)

    def img_callback(self, msg):
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
                        # message do testow tylko
                        """message = String()
                        message.data = f"id: {ids[0]} Dist: {round(distance, 2)} x:{round(tVec[i][0][0],1)} y: {round(tVec[i][0][1],1)}"
                        self.distance_pub.publish(message)"""

                        if ids[0] not in self.aruco_publishers:
                            self.aruco_publishers[ids[0]] = self.create_publisher(
                                PoseWithCovarianceStamped, f"/aruco/pose/nr{ids[0]}", 10
                            )

                        self.position_pub = self.aruco_publishers[ids[0]]
                        aruco_position = PoseWithCovarianceStamped()
                        aruco_position.header.stamp = self.get_clock().now().to_msg()
                        aruco_position.header.frame_id = "orbbec_astra_link"

                        # if PoseStamp then aruco_position.pose.position/orientation (one pose less)
                        aruco_position.pose.pose.position.x = tVec[i][0][0]
                        aruco_position.pose.pose.position.y = tVec[i][0][1]
                        aruco_position.pose.pose.position.z = tVec[i][0][2]

                        # Convert rotation vector to rotation matrix
                        rVec_matrix = tf_trans.rotation_matrix(
                            np.linalg.norm(rVec[i][0]),
                            rVec[i][0] / np.linalg.norm(rVec[i][0]),
                        )

                        # Convert rotation matrix to quaternion
                        quaternion = tf_trans.quaternion_from_matrix(rVec_matrix)

                        aruco_position.pose.pose.orientation.w = quaternion[0]
                        aruco_position.pose.pose.orientation.x = quaternion[1]
                        aruco_position.pose.pose.orientation.y = quaternion[2]
                        aruco_position.pose.pose.orientation.z = quaternion[3]

                        self.position_pub.publish(aruco_position)

                    except Exception as e:
                        self.get_logger().info(f"Publisher error: {e}")

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
