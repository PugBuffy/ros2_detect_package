import rclpy
import cv2  

from message_filters import ApproximateTimeSynchronizer, Subscriber
from bboxes_ex_msgs.msg import BoundingBoxes
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class Vizualizator(Node):
    def __init__(self):
        super().__init__('vizualizator')
        self.bridge = CvBridge()
        self.classes = ('pedestrian', 'car', 'cyclist', 'truck')
        self.colors = {
            'pedestrian':(0,255,0),
            'car' : (0, 0, 255),
            'cyclist' : (0, 0, 0),
            'truck' : (0, 255, 255)
        }

        subscribers = [
            Subscriber(self, Image, '/image'),
            Subscriber(self, BoundingBoxes, '/bounding_boxes'), 
            #Subscriber(self, BoundingBoxes, '/debug_detection')
        ]

        ats = ApproximateTimeSynchronizer(subscribers,
                                          queue_size=5,
                                          slop=0.1)
        ats.registerCallback(self.callback)

        self.publisher = self.create_publisher(Image, '/image_objects', 1)
        self.get_logger().info('Start vizualizator...')

    def callback(self, img, objects):
        img = self.bridge.imgmsg_to_cv2(img, 'bgr8')
        
        for object in objects.bounding_boxes:
            cv2.rectangle(img, [object.xmin, object.ymin], [object.xmax, object.ymax],
                          self.colors[object.class_id], 2)
            cv2.rectangle(img, [object.xmin, object.ymin - 20], [object.xmax, object.ymin],
                          self.colors[object.class_id], -1)

            cv2.putText(img, 
                        f'{object.id}:{object.class_id}',
                        (object.xmin, object.ymin - 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 
                        0.75, [0, 0, 0],
                        thickness=2)
            
        msg = self.bridge.cv2_to_imgmsg(img, 'bgr8')
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    vizualizator = Vizualizator()
    rclpy.spin(vizualizator)
    rclpy.shutdown()


if __name__ == '__main__':
    main()