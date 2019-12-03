from __future__ import print_function

from copy import deepcopy
from visualization_msgs.msg import Marker, InteractiveMarker, InteractiveMarkerControl
from interactive_markers.interactive_marker_server import InteractiveMarkerFeedback
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
import tf_transformations as tf
import numpy


def colorRGBA(r: float = 0, g: float = 0, b: float = 0, a: float = 1, **kwargs):
    return ColorRGBA(r=float(r), g=float(g), b=float(b), a=float(a), **kwargs)


def point(x: float = 0, y: float = 0, z: float = 0, **kwargs):
    return Point(x=float(x), y=float(y), z=float(z), **kwargs)


def quaternion(x: float = 0, y: float = 0, z: float = 0, w: float = 1, **kwargs):
    return Quaternion(x=float(x), y=float(y), z=float(z), w=float(w), **kwargs)


def sphere(radius=0.02, color=colorRGBA(1, 0, 1, 0.5), **kwargs):
    """Create a sphere marker"""
    scale = Vector3(x=radius, y=radius, z=radius)
    return Marker(type=Marker.SPHERE, scale=scale, color=color, **kwargs)


def cylinder(radius=0.02, len=0.1, color=colorRGBA(1, 0, 0), **kwargs):
    """Create a cylinder marker"""
    scale = Vector3(x=radius, y=radius, z=len)
    return Marker(type=Marker.CYLINDER, scale=scale, color=color, **kwargs)


def frame(T, scale=0.1, frame_id='world', ns='frame'):
    """Create a frame composed from three cylinders"""
    markers = []
    p = T[0:3, 3]

    defaults = dict(header=Header(frame_id=frame_id), ns=ns)

    xaxis = tf.quaternion_about_axis(numpy.pi / 2., [0, 1, 0])
    yaxis = tf.quaternion_about_axis(numpy.pi / 2., [-1, 0, 0])
    offset = numpy.array([0, 0, scale / 2.])

    m = cylinder(scale / 10., scale, color=colorRGBA(1, 0, 0, 1), id=0, **defaults)
    q = tf.quaternion_multiply(tf.quaternion_from_matrix(T), xaxis)
    m.pose.orientation = quaternion(*q)
    m.pose.position = point(*(p + tf.quaternion_matrix(q)[:3, :3].dot(offset)))
    markers.append(m)

    m = cylinder(scale / 10., scale, color=colorRGBA(0, 1, 0, 1), id=1, **defaults)
    q = tf.quaternion_multiply(tf.quaternion_from_matrix(T), yaxis)
    m.pose.orientation = quaternion(*q)
    m.pose.position = point(*(p + tf.quaternion_matrix(q)[:3, :3].dot(offset)))
    markers.append(m)

    m = cylinder(scale / 10., scale, color=colorRGBA(0, 0, 1, 1), id=2, **defaults)
    m.pose.orientation = quaternion(*tf.quaternion_from_matrix(T))
    m.pose.position = point(*(p + T[:3, :3].dot(offset)))
    markers.append(m)
    return markers


def add3DControls(im, markers, mode=InteractiveMarkerControl.MOVE_ROTATE_3D):
    # Create a control to move a (sphere) marker around with the mouse
    control = InteractiveMarkerControl()
    control.interaction_mode = mode
    control.markers.extend(markers)
    im.controls.append(control)


def addArrowControls(im, dirs='xyz'):
    # Create arrow controls to move the marker
    for dir in dirs:
        control = InteractiveMarkerControl()
        control.name = 'move_' + dir
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control.orientation.x = 1. if dir == 'x' else 0.
        control.orientation.z = 1. if dir == 'y' else 0.
        control.orientation.y = 1. if dir == 'z' else 0.
        control.orientation.w = 1.
        im.controls.append(control)


def addOrientationControls(im, dirs='xyz'):
    # Create controls to rotate the marker
    for dir in dirs:
        control = InteractiveMarkerControl()
        control.name = 'rotate_' + dir
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        control.orientation.x = 1.0 if dir == "x" else 0.0
        control.orientation.z = 1.0 if dir == "y" else 0.0
        control.orientation.y = 1.0 if dir == "z" else 0.0
        control.orientation.w = 1.0
        im.controls.append(control)


def createPose(T):
    if T.shape != (4, 4):  # if not 4x4 matrix: assume position vector
        Tnew = numpy.identity(4)
        Tnew[0:3, 3] = T
        T = Tnew
    return Pose(position=point(*(T[0:3, 3])), orientation=quaternion(*tf.quaternion_from_matrix(T)))


def addMarker(im_server, im, feedback_callback):
    # call feedback callback once to initialize target
    feedback_callback(InteractiveMarkerFeedback(marker_name=im.name, pose=im.pose))
    im_server.insert(im, feedback_callback=feedback_callback)


def poseMsgToTM(pose):
    q = pose.orientation
    p = pose.position
    T = tf.quaternion_matrix(numpy.array([q.x, q.y, q.z, q.w]))
    T[0:3, 3] = numpy.array([p.x, p.y, p.z])
    return T


def processFeedback(pose_callback):
    def process_marker_feedback(feedback):
        pose_callback(feedback.marker_name, poseMsgToTM(feedback.pose))
    return process_marker_feedback


def iMarker(T, markers=[], name='pose', mode=InteractiveMarkerControl.MOVE_ROTATE_3D, **kwargs):
    im = InteractiveMarker(name=name, pose=createPose(T), **kwargs)
    im.header.frame_id = 'world'
    if markers:
        add3DControls(im, markers, mode=mode)
    return im


def iPositionMarker(T, markers=[sphere()], name='pos', **kwargs):
    im = iMarker(T, markers, name, scale=0.2, description='Pos')
    addArrowControls(im)
    return im


def iPoseMarker(T, markers=[sphere()], name='pose'):
    im = iMarker(T, markers, name, scale=0.2, description='Pose 6D')
    addArrowControls(im)
    addOrientationControls(im)
    return im


def iPlaneMarker(pos, markers, name='plane', **kwargs):
    im = iMarker(pos, name=name, scale=0.2, description='Plane', **kwargs)
    if markers:
        add3DControls(im, markers)
    else:
        addArrowControls(im, dirs='z')
        addOrientationControls(im, dirs='xy')
    return im
