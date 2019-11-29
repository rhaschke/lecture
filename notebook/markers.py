from __future__ import print_function

from copy import deepcopy
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point, Quaternion
from tf import transformations as tf
import numpy


def cylinder(radius=0.02, len=0.1, color=ColorRGBA(1, 0, 0, 1), defaults=None):
    """Create a cylinder marker"""
    m = Marker() if defaults is None else deepcopy(defaults)
    m.type = Marker.CYLINDER
    m.scale.x = m.scale.y = radius
    m.scale.z = len
    m.color = color
    return m


def frame(T, scale=0.1, frame_id='world'):
    """Create a frame composed from three cylinders"""
    markers = []
    p = T[0:3, 3]

    m = Marker()
    m.ns = 'frame'
    m.header.frame_id = frame_id

    xaxis = tf.quaternion_about_axis(numpy.pi / 2., [0, 1, 0])
    yaxis = tf.quaternion_about_axis(numpy.pi / 2., [-1, 0, 0])
    offset = numpy.array([0, 0, scale / 2.])

    m = cylinder(scale / 10., scale, color=ColorRGBA(1, 0, 0, 1), defaults=m)
    m.id = 0
    q = tf.quaternion_multiply(tf.quaternion_from_matrix(T), xaxis)
    m.pose.orientation = Quaternion(*q)
    m.pose.position = Point(*(p + tf.quaternion_matrix(q)[:3, :3].dot(offset)))
    markers.append(m)

    m = cylinder(scale / 10., scale, color=ColorRGBA(0, 1, 0, 1), defaults=m)
    m.id = 1
    q = tf.quaternion_multiply(tf.quaternion_from_matrix(T), yaxis)
    m.pose.orientation = Quaternion(*q)
    m.pose.position = Point(*(p + tf.quaternion_matrix(q)[:3, :3].dot(offset)))
    markers.append(m)

    m = cylinder(scale / 10., scale, color=ColorRGBA(0, 0, 1, 1), defaults=m)
    m.id = 2
    m.pose.orientation = Quaternion(*tf.quaternion_from_matrix(T))
    m.pose.position = Point(*(p + T[:3, :3].dot(offset)))
    markers.append(m)
    return MarkerArray(markers)
