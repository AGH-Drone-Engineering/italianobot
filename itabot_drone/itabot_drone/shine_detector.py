import cv2
import numpy as np


def main():
    cap = cv2.VideoCapture('udpsrc port=5000 ! application/x-rtp,payload=96 ! rtph264depay ! h264parse ! nvh264dec ! videoconvert ! video/x-raw,format=BGR ! appsink sync=false', cv2.CAP_GSTREAMER)
    #cap = cv2.VideoCapture(0)

    while True:
        ret, src = cap.read()
        if not ret:
            break

        gray = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)

        gray = cv2.medianBlur(gray, 7)
        
        _, mask = cv2.threshold(gray, 250, 255, cv2.THRESH_BINARY)

        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5,5), np.uint8))

        src[mask > 0] = (0, 0, 255)

        cnts, _ = cv2.findContours(mask, 1, 2)
        cnt = cnts[0]
        M = cv2.moments(cnt)
        cx = M['m10'] / M['m00']
        cy = M['m01'] / M['m00']
        print(cx, cy)

        # xs, ys = np.meshgrid(np.arange(mask.shape[0]), np.arange(mask.shape[1]), indexing='ij')
        # if np.sum(mask) > 0:
            

            

        cv2.line(src, (int(cx), 0), (int(cx), src.shape[1]), (0, 0, 255))
        cv2.line(src, (0, int(cy)), (src.shape[0], int(cy)), (0, 0, 255))

        cv2.imshow("xd", src)
        if cv2.waitKey(1) == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
