import rclpy
import cv2
import torch
import numpy
import os 

from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.node import Node
from objects_msgs.msg import Object2D, Object2DArray
from models import TRTModule  # isort:skip
from models.torch_utils import det_postprocess
from models.utils import blob, letterbox


class Detector(Node):
    def __init__(self):
        super().__init__('detector')
        self.subscriber = self.create_subscription(Image, '/image', self.image_callback, 1)
        self.publisher = self.create_publisher(Object2DArray, "/detection", 1)
        self.get_logger().info('Start detector...')    
        
        self.bridge = CvBridge()
        self.device = torch.device("cuda:0")
        path_to_engine = os.path.abspath('src/detector/yolov8s.engine')
        self.engine = TRTModule(path_to_engine, self.device) 


    def image_callback(self, image: Image):
        msg = Object2DArray()    
        msg.header = image.header

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

        for (bbox, score, label) in zip(bboxes, scores, labels):
            object = Object2D() 
            bbox = bbox.round().int().tolist()

            object.x = bbox[0]
            object.y = bbox[1]
            object.width = bbox[2] - bbox[0]
            object.height = bbox[3] - bbox[1]
            object.score = score.float().tolist()
            object.label = label.int().tolist()

            msg.objects.append(object)

        self.publisher.publish(msg)
        
            
def main(args=None):
    rclpy.init(args=args)
    detector = Detector()
    rclpy.spin(detector)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
