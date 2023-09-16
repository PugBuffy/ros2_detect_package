#!/usr/bin/env python
# coding: utf-8
from __future__ import division, print_function

from copy import copy

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from objects_msgs.msg import ObjectArray

from .geometry import EDGES_INDICES, object_points
from .utils import object_color, object_label


def to_ColorRGBA(arr):
    return ColorRGBA(r=arr[2] / 255, g=arr[1] / 255, b=arr[0] / 255, a=1.0)


def create_markers(obj, header, color, label):
    p = obj.pose.position
    if p.x == 0 and p.y == 0 and p.z == 0:
        return []

    colora = copy(color)
    colora.a = 0.9
    ret = []

    m = Marker()
    m.header = header
    m.ns = "bbox"
    m.action = Marker.ADD
    m.type = Marker.CUBE
    m.color = copy(colora)
    m.color.a = 0.5
    m.scale = obj.size
    m.pose = obj.pose
    m.id = obj.id
    ret.append(m)

    m = Marker()
    m.header = header
    m.id = obj.id
    m.color = colora
    m.ns = "edges"
    m.type = Marker.LINE_LIST
    m.scale.x = 0.05
    m.pose.orientation.w = 1.0
    pts = object_points(obj)
    for i1, i2 in EDGES_INDICES:
        p1, p2 = pts[i1], pts[i2]
        m.points.append(Point(x=p1[0], y=p1[1], z=p1[2]))
        m.points.append(Point(x=p2[0], y=p2[1], z=p2[2]))
    ret.append(m)

    if label:
        m = Marker()
        m.header = header
        m.ns = "label"
        m.id = obj.id
        m.action = Marker.ADD
        m.type = Marker.TEXT_VIEW_FACING
        m.text = label
        m.scale.z = 0.5
        m.color = color
        m.pose.position = copy(obj.pose.position)
        m.pose.position.y -= obj.size.y / 2 + 0.2
        ret.append(m)

    return ret


DELETE_ALL_MARKERS = Marker(action=Marker.DELETEALL)


class Objects2Markers(Node):

    def __init__(self):
        super().__init__('objects2markers')
        self.declare_parameter('color', 'label')
        self.declare_parameter('label_fmt', '{label} {score:0.2f}')
        self.declare_parameter('classes', ['unknown'] * 10)

        self.color = self.get_parameter(
            'color').get_parameter_value().string_value
        self.label_fmt = self.get_parameter(
            'label_fmt').get_parameter_value().string_value
        self.classes = self.get_parameter(
            'classes').get_parameter_value().string_array_value

        self.obj_sub = self.create_subscription(ObjectArray, 'objects3d',
                                                self.objects_callback, 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'markers', 10)

    def objects_callback(self, objects):
        markers = MarkerArray()
        markers.markers.append(DELETE_ALL_MARKERS)
        for obj in objects.objects:
            label = object_label(obj, self.label_fmt, self.classes)
            color = to_ColorRGBA(object_color(obj, self.color))

            m = create_markers(obj, objects.header, color, label)
            markers.markers.extend(m)

        self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = Objects2Markers()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
