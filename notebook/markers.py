from visualization_msgs.msg import Marker, InteractiveMarker, InteractiveMarkerControl
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
from tf import transformations as tf
import numpy


def cone(halfOpenAngle, heigth=.1,  color=ColorRGBA(1, .2, 1, .2), **kwargs):
    twopi = numpy.pi *2
    radius = numpy.tan(halfOpenAngle)* heigth
    points = []
    numTriangels = 50
    for i in range(numTriangels):
        x1,y1 =  (radius * numpy.sin(twopi* i/numTriangels),    radius * numpy.cos(twopi*i/numTriangels))
        x2,y2 =  (radius * numpy.sin(twopi*(i+1)/numTriangels), radius * numpy.cos(twopi*(i+1)/numTriangels))

        points.append(Vector3(0,0,0))
        points.append(Vector3(x1,y1, heigth))
        points.append(Vector3(x2,y2, heigth))

    return Marker(type=Marker.TRIANGLE_LIST, points=points, color=color, **kwargs)


def sphere(radius=0.02, color=ColorRGBA(1, 0, 1, 0.5), **kwargs):
    """Create a sphere marker"""
    scale = Vector3(radius, radius, radius)
    return Marker(type=Marker.SPHERE, scale=scale, color=color, **kwargs)


def cylinder(radius=0.02, len=0.1, color=ColorRGBA(1, 0, 0, 1), **kwargs):
    """Create a cylinder marker"""
    scale = Vector3(radius, radius, len)
    return Marker(type=Marker.CYLINDER, scale=scale, color=color, **kwargs)


def box(size=Vector3(0.1, 0.1, 0.1), color=ColorRGBA(1, 1, 1, 0.5), **kwargs):
    """Create a box marker"""
    return Marker(type=Marker.CUBE, scale=size, color=color, **kwargs)


def plane(size=1.0, color=ColorRGBA(1, 1, 1, 0.5), **kwargs):
    """Create a plane (a flat box)"""
    return box(size=Vector3(size, size, 1e-3), color=color, **kwargs)


def frame(T, scale=0.1, frame_id='world'):
    """Create a frame composed from three cylinders"""
    markers = []
    p = T[0:3, 3]

    defaults = dict(header=Header(frame_id=frame_id), ns='frame')

    xaxis = tf.quaternion_about_axis(numpy.pi / 2., [0, 1, 0])
    yaxis = tf.quaternion_about_axis(numpy.pi / 2., [-1, 0, 0])
    offset = numpy.array([0, 0, scale / 2.])

    m = cylinder(scale / 10., scale, color=ColorRGBA(1, 0, 0, 1), id=0, **defaults)
    q = tf.quaternion_multiply(tf.quaternion_from_matrix(T), xaxis)
    m.pose.orientation = Quaternion(*q)
    m.pose.position = Point(*(p + tf.quaternion_matrix(q)[:3, :3].dot(offset)))
    markers.append(m)

    m = cylinder(scale / 10., scale, color=ColorRGBA(0, 1, 0, 1), id=1, **defaults)
    q = tf.quaternion_multiply(tf.quaternion_from_matrix(T), yaxis)
    m.pose.orientation = Quaternion(*q)
    m.pose.position = Point(*(p + tf.quaternion_matrix(q)[:3, :3].dot(offset)))
    markers.append(m)

    m = cylinder(scale / 10., scale, color=ColorRGBA(0, 0, 1, 1), id=2, **defaults)
    m.pose.orientation = Quaternion(*tf.quaternion_from_matrix(T))
    m.pose.position = Point(*(p + T[:3, :3].dot(offset)))
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
        control.name = "move_" + dir
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control.orientation.x = 1 if dir == 'x' else 0
        control.orientation.z = 1 if dir == 'y' else 0
        control.orientation.y = 1 if dir == 'z' else 0
        control.orientation.w = 1
        im.controls.append(control)


def addOrientationControls(im, dirs='xyz'):
    # Create controls to rotate the marker
    for dir in dirs:
        control = InteractiveMarkerControl()
        control.name = "rotate_" + dir
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        control.orientation.x = 1 if dir == 'x' else 0
        control.orientation.z = 1 if dir == 'y' else 0
        control.orientation.y = 1 if dir == 'z' else 0
        control.orientation.w = 1
        im.controls.append(control)


def createPose(T):
    if T.shape != (4, 4):  # if not 4x4 matrix: assume position vector
        Tnew = numpy.identity(4)
        Tnew[0:3, 3] = T
        T = Tnew
    return Pose(position=Point(*T[0:3, 3]), orientation=Quaternion(*tf.quaternion_from_matrix(T)))


def iPositionMarker(T, markers=[sphere()], name='pos'):
    im = InteractiveMarker()
    im.header.frame_id = "world"
    im.name = name
    im.description = "Pos"
    im.scale = 0.2
    im.pose = createPose(T)
    if markers:
        add3DControls(im, markers)
    addArrowControls(im)
    return im


def iPoseMarker(T, markers=[sphere()], name='pose'):
    im = iPositionMarker(T, markers, name)
    im.description = "Pose 6D"
    addOrientationControls(im)
    return im


def iPlaneMarker(pos, markers, name='plane'):
    im = InteractiveMarker()
    im.header.frame_id = "world"
    im.name = name
    im.description = "Plane"
    im.scale = 0.2
    im.pose = createPose(pos)
    if markers:
        add3DControls(im, markers)
    else:
        addArrowControls(im, dirs='z')
        addOrientationControls(im, dirs='xy')
    return im
