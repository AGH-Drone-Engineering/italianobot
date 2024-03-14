import os
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

CHESS_BOARD_DIM = (9, 6)

n = 0  # image_counter

# checking if images dir is exist not, if not then create images directory
image_dir_path = "images"

CHECK_DIR = os.path.isdir(image_dir_path)
# if directory does not exist create
if not CHECK_DIR:
    os.makedirs(image_dir_path)
    print(f'"{image_dir_path}" Directory is created')
else:
    print(f'"{image_dir_path}" Directory already Exists.')

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)


def detect_checker_board(image, grayImage, criteria, boardDimension):
    ret, corners = cv2.findChessboardCorners(grayImage, boardDimension)
    if ret:
        corners1 = cv2.cornerSubPix(grayImage, corners, (3, 3), (-1, -1), criteria)
        image = cv2.drawChessboardCorners(image, boardDimension, corners1, ret)

    return image, ret

gstreamer_pipe = "udpsrc port=5000 ! application/x-rtp,payload=96 ! rtph264depay ! h264parse ! queue ! vah264dec ! videoconvert ! video/x-raw,format=BGR ! appsink sync=false"


class ImageSubscriber:
    # def __init__(self):
    #     super().__init__('image_subscriber')
    #     qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
    #     self.subscription = self.create_subscription(
    #         Image,
    #         '/camera/drone/image',
    #         self.image_callback,
    #         qos_profile
    #     )
        # self.subscription  # prevent unused variable warning
    def __init__(self):
        pass
        
    def image_callback(self):
        self.cap = cv2.VideoCapture(gstreamer_pipe, cv2.CAP_GSTREAMER)
        n =0
        while 1:
            
            ret, frame = self.cap.read()
            
            
            copyFrame = frame.copy()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            print("debug1")

            image, board_detected = detect_checker_board(frame, gray, criteria, CHESS_BOARD_DIM)

            cv2.putText(
                frame,
                f"saved_img : {n}",
                (30, 40),
                cv2.FONT_HERSHEY_PLAIN,
                1.4,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )
            cv2.imshow("frame", frame)
            cv2.imshow("copyFrame", copyFrame)

            key = cv2.waitKey(1)

            if key == ord("q"):
                cv2.destroyAllWindows()
                self.destroy_node()
                rclpy.shutdown()
            elif key == ord("s") and board_detected:
                # storing the checker board image
                cv2.imwrite(f"{image_dir_path}/photo{n}.png", copyFrame)

                print(f"saved image number {n}")
                n += 1  # incrementing the image counter


def main(args=None):
    global n
    n=0
    image_subscriber = ImageSubscriber()
    image_subscriber.image_callback()


if __name__ == '__main__':
    main()
