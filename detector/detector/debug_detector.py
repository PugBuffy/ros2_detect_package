import rclpy
import cv2
import torch
import numpy 
import os

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.node import Node
from bboxes_ex_msgs.msg import BoundingBox, BoundingBoxes
from models import TRTModule  # isort:skip
from models.torch_utils import det_postprocess
from models.utils import blob, letterbox
from math import sqrt


class Detector(Node):
    def __init__(self):
        super().__init__('debug_detector')
        self.subscriber = self.create_subscription(Image, '/image', self.image_callback, 1)
        self.publisher = self.create_publisher(BoundingBoxes, "/debug_detection", 1)
        self.get_logger().info('Start detector...')    
        
        self.bridge = CvBridge()
        self.device = torch.device("cuda:0")
        path_to_engine = os.path.abspath('src/detector/yolov8s.engine')
        self.engine = TRTModule(path_to_engine, self.device) 

        self.classes = ('pedestrian', 'car', 'cyclist', 'truck') 

    def image_callback(self, image: Image):
        image = self.bridge.imgmsg_to_cv2(image, 'bgr8')

        H, W = self.engine.inp_info[0].shape[-2:]
        self.engine.set_desired(['num_dets', 'bboxes', 'scores', 'labels'])
        image, ratio, dwdh = letterbox(image, (W, H))
        rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        tensor = blob(rgb, return_seg=False)
        dwdh = torch.asarray(dwdh * 2, dtype=torch.float32, device=self.device)
        tensor = torch.asarray(tensor, device=self.device)
            
        data = self.engine(tensor)

        bboxes, scores, labels = det_postprocess(data)
        bboxes -= dwdh
        bboxes /= ratio

        msg = BoundingBoxes()

        for (bbox, score, label) in zip(bboxes, scores, labels):
            object = BoundingBox()
            bbox = bbox.round().int().tolist()

            object.probability = score.float().tolist()
            object.xmin = abs(bbox[1])
            object.ymin = abs(bbox[0])
            
            object.xmax = bbox[3]
            object.ymax = bbox[2]
            object.img_height = 2160
            object.img_width = 3840
            object.center_dist = int(sqrt((bbox[2] - bbox[0]) ** 2 + (bbox[3] - bbox[1]) ** 2) / 2)
            object.class_id = self.classes[label.int().tolist()]
            object.class_id_int = label.int().tolist()

            msg.bounding_boxes.append(object)

        self.publisher.publish(msg)
        
            
def main(args=None):
    rclpy.init(args=args)
    detector = Detector()
    rclpy.spin(detector)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
