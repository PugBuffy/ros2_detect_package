import rclpy
import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.node import Node


class Camera(Node):
    def __init__(self):
        super().__init__('camera')
        self.publisher = self.create_publisher(Image, '/image', 1)
        self.bridge = CvBridge()
        self.declare_parameter('video_file', '/dev/video1')
        self.cap = cv2.VideoCapture(self.get_parameter('video_file').get_parameter_value().string_value)
        self.timer = self.create_timer(0, self.callback) 
        self.get_logger().info('Start camera...')

    def callback(self):
        if self.get_parameter('video_file').get_parameter_value().string_value == '/dev/video0':
            image = self.cap.read()[1]
            image = self.bridge.cv2_to_imgmsg(image, 'bgr8')
            self.publisher.publish(image)
        else:
            while self.cap.isOpened():
                res, image = self.cap.read()
            
                if res:
                    image = self.bridge.cv2_to_imgmsg(image, 'bgr8')
                    self.publisher.publish(image)
                else:
                    self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    continue
        

def main(args=None):
    rclpy.init(args=args)
    camera = Camera()
    rclpy.spin(camera)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
