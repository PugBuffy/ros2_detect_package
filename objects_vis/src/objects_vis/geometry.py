from collections import namedtuple

import numpy as np
from geometry_msgs.msg import Point, Pose, Quaternion, Transform, Vector3
from sensor_msgs.msg import CameraInfo
from scipy.spatial.transform import Rotation


def Rt_from_tq(t, q):
    Rt = np.zeros((4, 4), np.float64)
    Rt[:3, :3] = Rotation.from_quat((q.x, q.y, q.z, q.w)).as_matrix()
    Rt[:3, 3] = t.x, t.y, t.z
    return Rt


def Rt_from_Pose(pose):
    return Rt_from_tq(pose.position, pose.orientation)


def Rt_from_Transform(tf):
    return Rt_from_tq(tf.translation, tf.rotation)


def Pose_from_Rt(Rt):
    pose = Pose()
    q = pose.orientation
    t = pose.position
    q.x, q.y, q.z, q.w = Rotation.from_matrix(Rt[:3, :3]).as_quat()
    t.x, t.y, t.z = Rt[:3, 3]
    return pose


def transform_object(obj, tf):
    obj.pose = Pose_from_Rt(
        np.dot(Rt_from_Transform(tf), Rt_from_Pose(obj.pose)))


def set_object_yaw(obj, yaw, camera_frame=False):
    if camera_frame:
        R = np.array([
            [-np.sin(yaw), -np.cos(yaw), 0.0],
            [0.0, 0.0, -1.0],
            [np.cos(yaw), -np.sin(yaw), 0.0],
        ])
        q = Rotation.from_matrix(R).as_quat()
        obj.pose.orientation.x = q[0]
        obj.pose.orientation.y = q[1]
        obj.pose.orientation.z = q[2]
        obj.pose.orientation.w = q[3]
    else:
        obj.pose.orientation.x = 0
        obj.pose.orientation.y = 0
        obj.pose.orientation.z = np.sin(yaw / 2)
        obj.pose.orientation.w = np.cos(yaw / 2)


def get_object_yaw(obj, camera_frame=False):
    if camera_frame:
        q = obj.pose.orientation
        R = Rotation.from_quat((q.x, q.y, q.z, q.w)).as_matrix()
        yaw = np.arctan2(-R[0, 0], -R[0, 1])
    else:
        yaw = np.arctan2(obj.pose.orientation.z, obj.pose.orientation.w)
    return yaw


def object_points(obj, Rt=None):
    dx, dy, dz = obj.size.x / 2, obj.size.y / 2, obj.size.z / 2
    # additional axis with ones for matrix multiplication
    pts = np.array([
        [-dx, -dy, -dz, 1],  # 0: back bottom right
        [+dx, -dy, -dz, 1],  # 1: front bottom right
        [+dx, +dy, -dz, 1],  # 2: front bottom left
        [-dx, +dy, -dz, 1],  # 3: back bottom left
        [-dx, -dy, +dz, 1],  # 4: back top right
        [+dx, -dy, +dz, 1],  # 5: front top right
        [+dx, +dy, +dz, 1],  # 6: front top left
        [-dx, +dy, +dz, 1],  # 7: back top left
    ])
    if Rt is None:
        Rt = np.eye(4)
    res = np.dot(np.dot(Rt, Rt_from_Pose(obj.pose)), pts.T).T[:, :3]
    return res


EDGES_INDICES = [(0, 1), (1, 2), (2, 3), (3, 0), (0, 4), (1, 5), (2, 6),
                 (3, 7), (4, 5), (5, 6), (6, 7), (7, 4), (1, 6), (2, 5)]

CameraInfoMatrices = namedtuple('CameraInfoMatrices', 'K, D, R, P')


def cam_info_to_matrices(cam_info):
    K = np.array(cam_info.k).reshape(3, 3)
    D = np.array(cam_info.d)
    R = np.array(cam_info.r).reshape(3, 3)
    P = np.array(cam_info.p).reshape(3, 4)
    return CameraInfoMatrices(K=K, D=D, R=R, P=P)