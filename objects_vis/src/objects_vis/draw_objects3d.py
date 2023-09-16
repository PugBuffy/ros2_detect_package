#!/bin/env python
from __future__ import division, print_function

import cv2 as cv
import rclpy
import tf2_ros
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from objects_msgs.msg import ObjectArray
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

from .utils import draw_object3d, object_color


class DrawObjects3D(Node):

    def __init__(self):
        super().__init__('draw_objects3d')
        self.bridge = CvBridge()

        self.declare_parameter('queue_size', 5)
        self.declare_parameter('visualize', False)
        self.declare_parameter('color', 'label')
        self.declare_parameter('rectified', False)
        self.declare_parameter('camera_frame', '')

        queue_size = self.get_parameter(
            'queue_size').get_parameter_value().integer_value
        self.visualize = self.get_parameter(
            'visualize').get_parameter_value().bool_value
        self.color = self.get_parameter(
            'color').get_parameter_value().string_value
        self.rectified = self.get_parameter(
            'rectified').get_parameter_value().bool_value
        self.camera_frame = self.get_parameter(
            'camera_frame').get_parameter_value().string_value

        self.cam_info = None
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.cam_sub = self.create_subscription(CameraInfo, 'camera_info',
                                                self.cam_callback, 1)
        subscribers = [
            Subscriber(self, Image, 'image'),
            Subscriber(self, ObjectArray, 'objects3d')
        ]
        ats = ApproximateTimeSynchronizer(subscribers,
                                          queue_size=queue_size,
                                          slop=0.1)
        ats.registerCallback(self.callback)

        self.output = self.create_publisher(Image, 'image_objects3d', 1)

        if self.visualize:
            cv.namedWindow(self.output.topic_name)

    def cam_callback(self, cam_info):
        self.cam_info = cam_info

    def callback(self, img_msg, objects):
        if self.cam_info is None:
            self.get_logger().warn('No camera info!')
            return

        camera_frame = img_msg.header.frame_id
        if self.camera_frame:
            camera_frame = self.camera_frame

        tf = self.tf_buffer.lookup_transform(camera_frame,
                                             objects.header.frame_id,
                                             objects.header.stamp)

        img = self.bridge.imgmsg_to_cv2(img_msg, 'bgr8')

        for obj in objects.objects:
            color = object_color(obj, self.color)
            draw_object3d(img, obj, self.cam_info, color, tf.transform,
                          self.rectified)

        out = self.bridge.cv2_to_imgmsg(img, 'bgr8')
        out.header = img_msg.header
        self.output.publish(out)

        if self.visualize:
            cv.imshow(self.output.topic_name, img)


def main(args=None):
    rclpy.init(args=args)
    node = DrawObjects3D()
    while rclpy.ok():
        rclpy.spin_once(node)
        if node.visualize:
            cv.waitKey(1)
    node.destroy_node()
    rclpy.shutdown()
