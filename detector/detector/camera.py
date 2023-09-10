import rclpy
import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.node import Node


class Camera(Node):
    def __init__(self):
        super().__init__('camera')
        self.publisher = self.create_publisher(Image, '/image', 1)
        self.cap = cv2.VideoCapture('/dev/video0')
        self.bridge = CvBridge()
        self.get_logger().info('Start...')
        self.timer = self.create_timer(0, self.callback) 

    def callback(self):
        image = self.cap.read()[1]
        image = self.bridge.cv2_to_imgmsg(image, 'bgr8')
        self.publisher.publish(image)
        self.get_logger().info('sending')
        

def main(args=None):
    rclpy.init(args=args)
    camera = Camera()
    rclpy.spin(camera)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
