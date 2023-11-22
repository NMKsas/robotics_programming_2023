import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import numpy as np
import cv2
import imutils
from sensor_msgs.msg import Image

class FaceTracker(Node):

    def __init__(self):
        # Initialize the node
        super().__init__('FaceTracker')
        self.bridge = CvBridge()
        # Initialize the subscriber
        self.subscription_ = self.create_subscription(Image, '/image_raw', self.listener_callback, 10)
        self.subscription_  # prevent unused variable warning
        #timer_period = 0.1  # seconds
        #self.timer = self.create_timer(timer_period, self.timer_callback)
        self.K = False
        self.first_image = False
        self.p1 = []
        self.frame_count = 0
        self.update_interval = 100
        self.gray_prev = None
        self.frame = None


    def listener_callback(self, msg):

        self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.get_logger().info("Image Received")
        self.frame_count += 1

        gray, img = self.prep(self.frame)
        if self.K is False:
            self.gray_prev = gray.copy()
            self.p0, self.faces, img = self.get_trackable_points(gray, img)

        # update trackable points is done every 100 frames, or if only 10 trackable points are left
        if self.frame_count % self.update_interval == 0 or len(self.p0) <= 10:
            self.p0, self.faces, img = self.get_trackable_points(gray, img)
            self.gray_prev = gray.copy()

        else:
            for (x, y, w, h) in self.faces:
                cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 2)
                self.p1 = self.do_track_face(self.gray_prev, gray, self.p0)
            for i in self.p1:
                cv2.drawMarker(img, (int(i[0]), int(i[1])), [255, 0, 0], 0, 8)

        self.frame = img
        self.K = True
        self.timer_callback()
        return self.frame

    def timer_callback(self):
        if self.K == True:
            cv2.imshow("Tracking", self.frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                self.get_logger().info('Closing stream window..')
                self.K = False
                cv2.destroyAllWindows()
                self.stop_stream()


    @staticmethod
    def prep(image):
        # resize, flip and turn image to grayscale
        img = imutils.resize(image, width=600)
        img = cv2.flip(img, 1)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        return gray, img

    @staticmethod
    def get_trackable_points(gray, img):
        face_cascade = cv2.CascadeClassifier(
            '/home/sasnmk/ros/EX4/src/face_tracking/face_tracking/haarcascade_frontalface_default.xml')
        faces = face_cascade.detectMultiScale(gray, 1.1, 5)
        rectangle_color = (255, 0, 0)
        p0 = []

        if len(faces) != 0:
            for (x, y, w, h) in faces:
                start_point = (x, y)
                end_point = (x + w, y + h)
                img = cv2.rectangle(img, start_point, end_point, rectangle_color, 1)

                roi_gray = gray[y:y + h, x:x + w]

                new_p0 = cv2.goodFeaturesToTrack(roi_gray, maxCorners=70, qualityLevel=0.001, minDistance=5)

                new_p0 = new_p0 + np.array([x, y], dtype=np.float32)
                for p in new_p0:
                    p0.append(p)

            p0 = np.array(p0, dtype=np.float32)
            p0 = p0.reshape(-1, p0.shape[-1])

        return p0, faces, img

    @staticmethod
    def do_track_face(gray_prev, gray, p0):
        p1, is_found, err = cv2.calcOpticalFlowPyrLK(gray_prev, gray, p0, None, winSize=(31, 31), maxLevel=10,
                                                     criteria=(
                                                         cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.03),
                                                     flags=cv2.OPTFLOW_LK_GET_MIN_EIGENVALS,
                                                     minEigThreshold=0.00025)
        is_found = is_found.flatten()
        return p1[is_found == 1]

    def stop_stream(self):
        self.get_logger().info('Stopping the stream ...')
        quit()

def main():
    rclpy.init(args=None)
    image_subscriber = FaceTracker()
    try:
        while rclpy.ok():
            rclpy.spin(image_subscriber)

    except KeyboardInterrupt:
        image_subscriber.stop_stream()
        image_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
