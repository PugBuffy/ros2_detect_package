import rclpy
import cv2
import torch
import numpy 

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.node import Node
from objects_msgs.msg import DebugObject2D, DebugObject2DArray
from models import TRTModule  # isort:skip
from models.torch_utils import det_postprocess
from models.utils import blob, letterbox
from math import sqrt


class Detector(Node):
    def __init__(self):
        super().__init__('detector')
        self.subscriber = self.create_subscription(Image, '/image', self.image_callback, 1)
        self.image = numpy.ndarray
        self.publisher = self.create_publisher(DebugObject2DArray, "/detection", 1)
        self.timer = self.create_timer(0, self.publish_bbox)
        self.get_logger().info('Start detector...')    
        
        self.bridge = CvBridge()
        self.device = torch.device("cuda:0")
        #path_to_engine = "/root/src/detector/yolov8s.engine" # for docker 
        path_to_engine = "/home/sergey/workspace/ros2_ws/src/detector/yolov8s.engine"
        self.engine = TRTModule(path_to_engine, self.device)
        self.classes = ('pedestrian', 'car', 'cyclist', 'truck') 

    def image_callback(self, data: Image):
        self.image = self.bridge.imgmsg_to_cv2(data, 'bgr8')

    def publish_bbox(self):
        if type(self.image) == numpy.ndarray:
            H, W = self.engine.inp_info[0].shape[-2:]
            self.engine.set_desired(['num_dets', 'bboxes', 'scores', 'labels'])
  
            self.image, ratio, dwdh = letterbox(self.image, (W, H))
            rgb = cv2.cvtColor(self.image, cv2.COLOR_BGR2RGB)
            tensor = blob(rgb, return_seg=False)
            dwdh = torch.asarray(dwdh * 2, dtype=torch.float32, device=self.device)
            tensor = torch.asarray(tensor, device=self.device)
            
            data = self.engine(tensor)

            bboxes, scores, labels = det_postprocess(data)
            bboxes -= dwdh
            bboxes /= ratio

            msg = DebugObject2DArray()

            for (bbox, score, label) in zip(bboxes, scores, labels):
                object = DebugObject2D()
                bbox = bbox.round().int().tolist()

                object.probability = score.float().tolist()
                object.xmin = bbox[0]
                object.ymin = bbox[1]
                object.xmax = bbox[2]
                object.ymax = bbox[3]
                object.img_height = self.image.shape[0]
                object.img_width = self.image.shape[1]
                object.center_dist = int(sqrt((bbox[2] - bbox[0]) ** 2 + (bbox[3] - bbox[1]) ** 2) / 2)
                object.class_id = self.classes[label.int().tolist()]
                object.class_id_int = label.int().tolist()

                msg.objects.append(object)

            self.publisher.publish(msg)
            self.get_logger().info('publishing')
        
            
def main(args=None):
    rclpy.init(args=args)
    detector = Detector()
    rclpy.spin(detector)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
