import rclpy
import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.node import Node
from std_msgs.msg import Header


class Camera(Node):
    def __init__(self):
        super().__init__('camera')
        self.publisher = self.create_publisher(Image, '/image', 1)
        self.bridge = CvBridge()
        self.declare_parameter('video_file', '/dev/video0')
        self.declare_parameter('timer_period_sec', 0)
        
        video_file = self.get_parameter('video_file').get_parameter_value().string_value
        timer_period_sec = self.get_parameter('timer_period_sec').get_parameter_value().integer_value

        self.cap = cv2.VideoCapture(video_file)
        self.timer = self.create_timer(timer_period_sec, self.callback) 
        self.get_logger().info('Start camera...')

    def callback(self):
        if self.get_parameter('video_file').get_parameter_value().string_value == '/dev/video0':
            image = self.cap.read()[1]

            header = Header()
            header.frame_id = 'camera'
            header.stamp = self.get_clock().now().to_msg()

            image = self.bridge.cv2_to_imgmsg(image, 'bgr8', header)
            self.publisher.publish(image)
        else:
            while self.cap.isOpened():
                res, image = self.cap.read()

                if res:
                    header = Header()
                    header.frame_id = 'video'
                    header.stamp = self.get_clock().now().to_msg()

                    image = self.bridge.cv2_to_imgmsg(image, 'bgr8', header)
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
