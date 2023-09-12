import rclpy
import cv2
import numpy
import random

from sensor_msgs.msg import Image
from rclpy.node import Node
from cv_bridge import CvBridge
from objects_msgs.msg import Object2DArray


class Vizualizator(Node):
    def __init__(self):
        super().__init__('vizualizator')
        self.subscriber_img = self.create_subscription(Image, '/image', self.image_callback, 1)
        self.image = numpy.ndarray
        self.subscriber_bbox = self.create_subscription(Object2DArray, '/detection', self.bbox_callback ,1)
        self.bbox = Object2DArray
        self.bridge = CvBridge()
        self.classes = ('pedestrian', 'car', 'cyclist', 'truck')
        self.colors = {cls: [random.randint(0, 255) for _ in range(3)] for cls in self.classes}
        self.get_logger().info('Start vizualizator...')

    def image_callback(self, data: Image):
        self.image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        
    def bbox_callback(self, array: Object2DArray):
        if type(self.image) == numpy.ndarray:
            for object in array.objects:
                self.get_logger().info(self.classes[object.label])
                cv2.rectangle(self.image, (object.x, object.y), (object.width + object.x, object.height + object.y )
                              ,self.colors[self.classes[object.label]], 2)
                cv2.putText(self.image,
                        f'{self.classes[object.label]}:{object.score:.3f}', (object.x, object.y - 2),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.75, [225, 255, 255],
                        thickness=2)

            cv2.imshow('camera', self.image)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    vizualizator = Vizualizator()
    rclpy.spin(vizualizator)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    