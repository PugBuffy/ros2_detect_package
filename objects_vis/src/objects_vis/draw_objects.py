#!/bin/env python
from __future__ import division, print_function

import cv2 as cv
import rclpy
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from objects_msgs.msg import Object2DArray
from rclpy.node import Node
from sensor_msgs.msg import Image

from .utils import draw_object


class DrawObjects(Node):

    def __init__(self):
        super().__init__('draw_objects')
        self.bridge = CvBridge()

        self.declare_parameter('queue_size', 5)
        self.declare_parameter('visualize', False)
        self.declare_parameter('color', 'label')
        self.declare_parameter('label_fmt', '{label} {score:0.2f}')
        self.declare_parameter('font_scale', 1.0)
        self.declare_parameter('classes', ['unknown'] * 10)

        queue_size = self.get_parameter(
            'queue_size').get_parameter_value().integer_value
        self.visualize = self.get_parameter(
            'visualize').get_parameter_value().bool_value
        self.color = self.get_parameter(
            'color').get_parameter_value().string_value
        self.label_fmt = self.get_parameter(
            'label_fmt').get_parameter_value().string_value
        self.font_scale = self.get_parameter(
            'font_scale').get_parameter_value().double_value
        self.classes = self.get_parameter(
            'classes').get_parameter_value().string_array_value

        subscribers = [
            Subscriber(self, Image, '/image'),
            Subscriber(self, Object2DArray, '/bounding_boxes')
        ]
        
        ats = ApproximateTimeSynchronizer(subscribers,
                                          queue_size=queue_size,
                                          slop=0.1)
        ats.registerCallback(self.callback)

        self.output = self.create_publisher(Image, 'image_objects', 1)

        if self.visualize:
            cv.namedWindow(self.output.topic_name)

    def callback(self, img_msg, objects):
        img = self.bridge.imgmsg_to_cv2(img_msg, 'bgr8')

        for obj in objects.objects:
            draw_object(img, obj, self.color, self.label_fmt, self.classes,
                        self.font_scale)

        out = self.bridge.cv2_to_imgmsg(img, 'bgr8')
        out.header = img_msg.header
        self.output.publish(out)

        if self.visualize:
            cv.imshow(self.output.topic_name, img)


def main(args=None):
    rclpy.init(args=args)
    node = DrawObjects()
    while rclpy.ok():
        rclpy.spin_once(node)
        if node.visualize:
            cv.waitKey(1)
    node.destroy_node()
    rclpy.shutdown()
