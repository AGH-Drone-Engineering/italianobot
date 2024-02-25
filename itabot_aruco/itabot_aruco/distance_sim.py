import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import cv2
from cv2 import aruco
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import TransformStamped
import tf_transformations as tf_trans
import tf2_ros


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
            Image,
            "/camera/color/image_raw",
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

        # pose with covariance:
        self.aruco_publishers = dict()

        # tf broadcaster:
        self.aruco_broadcaster = tf2_ros.TransformBroadcaster(self)

        # do testow awaryjnie (potem do usuniecia):
        self.distance_pub = self.create_publisher(String, "/distance", 10)

        # Aruco code parameters:

        self.MARKER_SIZE = 10  # centimeters
        self.marker_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
        self.param_markers = aruco.DetectorParameters_create()

    def calib_callback(self, msg):
        # Camera calibration from topic:

        self.cam_mat = np.array(msg.k).reshape((3, 3))
        self.dist_coef = np.array(msg.d)

    def rviz2_axes(self, rVec, i):
        R, _ = cv2.Rodrigues(rVec[i][0])
        R_y = np.array(
            [
                [np.cos(np.pi / 2), 0, np.sin(np.pi / 2)],
                [0, 1, 0],
                [-np.sin(np.pi / 2), 0, np.cos(np.pi / 2)],
            ]
        )
        R_x = np.array(
            [
                [1, 0, 0],
                [0, np.cos(-np.pi / 2), -np.sin(-np.pi / 2)],
                [0, np.sin(-np.pi / 2), np.cos(-np.pi / 2)],
            ]
        )
        R_rotated = np.dot(R, R_y)
        R_rotated = np.dot(R_rotated, R_x)
        rVec_rotated, _ = cv2.Rodrigues(R_rotated)
        rVec[i] = rVec_rotated.T
    
    def img_callback(self, msg):
        try:

            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

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
                    
                    rVec2 = rVec.copy()
                    self.rviz2_axes(rVec2, i)
                    
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
                        frame,
                        self.cam_mat,
                        self.dist_coef,
                        rVec2[i],
                        tVec[i],
                        4 / 100,
                        4,
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
                        f"x:{round(tVec[i][0][2],1)} y:{round(-tVec[i][0][0],1)} z: {round(-tVec[i][0][1],1)} ",
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

                        # Convert rotation vector to rotation matrix
                        rVec_matrix = tf_trans.rotation_matrix(
                            np.linalg.norm(rVec[i][0]),
                            rVec[i][0] / np.linalg.norm(rVec[i][0]),
                        )

                        # Convert rotation matrix to quaternion

                        quaternion = tf_trans.quaternion_from_matrix(rVec_matrix)

                        # PoseWithCovarianceStamped:
                        if ids[0] not in self.aruco_publishers:
                            self.aruco_publishers[ids[0]] = self.create_publisher(
                                PoseWithCovarianceStamped, f"/aruco/pose/nr{ids[0]}", 10
                            )

                        aruco_position = PoseWithCovarianceStamped()
                        aruco_position.header.stamp = self.get_clock().now().to_msg()
                        aruco_position.header.frame_id = "orbbec_astra_link"
                        self.position_pub = self.aruco_publishers[ids[0]]

                        aruco_position.pose.pose.position.x = tVec[i][0][2]
                        aruco_position.pose.pose.position.y = -tVec[i][0][0]
                        aruco_position.pose.pose.position.z = -tVec[i][0][1]

                        aruco_position.pose.pose.orientation.w = quaternion[0]
                        aruco_position.pose.pose.orientation.x = quaternion[1]
                        aruco_position.pose.pose.orientation.y = quaternion[2]
                        aruco_position.pose.pose.orientation.z = quaternion[3]

                        self.position_pub.publish(aruco_position)

                        # TF_broadcast:
                        aruco_ekf = TransformStamped()
                        aruco_ekf.header.stamp = self.get_clock().now().to_msg()
                        aruco_ekf.header.frame_id = "orbbec_astra_link"
                        aruco_ekf.child_frame_id = f"aruco_marker_{ids[0]}"

                        aruco_ekf.transform.translation.x = tVec[i][0][2]
                        aruco_ekf.transform.translation.y = -tVec[i][0][0]
                        aruco_ekf.transform.translation.z = -tVec[i][0][1]

                        aruco_ekf.transform.rotation.w = quaternion[0]
                        aruco_ekf.transform.rotation.x = quaternion[1]
                        aruco_ekf.transform.rotation.y = quaternion[2]
                        aruco_ekf.transform.rotation.z = quaternion[3]

                        self.aruco_broadcaster.sendTransform(aruco_ekf)

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
