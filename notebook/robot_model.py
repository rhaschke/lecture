from __future__ import print_function

import numpy
import rospy
import xml.dom.minidom
from tf import transformations as tf


def get_value(xml, child=None, attribute=None):
    if child is not None:
        xml = xml.getElementsByTagName(child)[0]
    if attribute is not None:
        return xml.getAttribute(attribute)


def parse_vector(s):
    return numpy.array([float(v) for v in s.split(' ')])


def hat(p):
    return numpy.array([[0, -p[2], p[1]],
                        [p[2], 0, -p[0]],
                        [-p[1], p[0], 0]])


def adjoint(T, inverse=False):
    R = T[0:3, 0:3]
    p = T[0:3, 3]
    if not inverse:
        return numpy.block([[R, hat(p).dot(R)], [numpy.zeros((3, 3)), R]])
    else:
        return numpy.block([[R.T, R.T.dot(hat(-p))], [numpy.zeros((3, 3)), R.T]])


class Mimic():
    def __init__(self, tag):
        self.joint = tag.getAttribute('joint')
        if tag.hasAttribute('multiplier'):
            self.multiplier = float(tag.getAttribute('multiplier'))
        else:
            self.multiplier = 1.0
        if tag.hasAttribute('offset'):
            self.offset = float(tag.getAttribute('offset'))
        else:
            self.offset = 0.0


class Joint():
    fixed = 0
    revolute = 1
    continuous = 1
    prismatic = 2
    floating = 3

    def __init__(self, arg):
        if isinstance(arg, xml.dom.minidom.Element):
            self._init_from_xml(arg)
        else:
            self._init_from_pose(arg)

    def _init_from_xml(self, tag):
        self.jtype = getattr(Joint, tag.getAttribute('type'))
        self.active = self.jtype in [Joint.revolute, Joint.prismatic]
        self.name = tag.getAttribute('name')
        self.parent = get_value(tag, 'parent', 'link')
        self.child = get_value(tag, 'child', 'link')
        self.T = tf.euler_matrix(*parse_vector(get_value(tag, 'origin', 'rpy')), axes='sxyz')
        self.T[0:3, 3] = parse_vector(get_value(tag, 'origin', 'xyz'))
        self.mimic = None
        if self.active:
            self.axis = parse_vector(get_value(tag, 'axis', 'xyz'))
            try:
                self.min = float(get_value(tag, 'limit', 'lower'))
                self.max = float(get_value(tag, 'limit', 'upper'))
            except IndexError:
                raise Exception("Joint %s has not limits" % self.name)

            mimic = tag.getElementsByTagName('mimic')
            self.mimic = Mimic(mimic[0]) if mimic else None

    def _init_from_pose(self, transform):
        self.jtype = Joint.fixed
        self.active = False
        self.name = transform.child_frame_id
        self.parent = transform.header.frame_id
        self.child = transform.child_frame_id
        q = transform.transform.rotation
        p = transform.transform.translation
        self.T = tf.quaternion_matrix(numpy.array([q.x, q.y, q.z, q.w]))
        self.T[0:3, 3] = numpy.array([p.x, p.y, p.z])
        self.mimic = None


class RobotModel():
    def __init__(self, param='robot_description'):
        self.links = {}  # map link to its parent joint
        self.joints = {}  # map joint name to joint instance
        self.active_joints = []  # active joints

        description = rospy.get_param(param)
        doc = xml.dom.minidom.parseString(description)
        robot = doc.getElementsByTagName('robot')[0]
        for tag in robot.getElementsByTagName('joint'):
            self._add(Joint(tag))

    def _add(self, joint):
        self.joints[joint.name] = joint
        if joint.active and joint.mimic is None:
            self.active_joints.append(joint)
        if joint.mimic is not None:
            joint.mimic.joint = self.joints[joint.mimic.joint]  # replace name with instance
        self.links[joint.child] = joint
        if joint.parent not in self.links:
            self.links[joint.parent] = None

    def fk(self, link, joints):
        def value(joint):
            """Get joint value from joints, considering mimic joints"""
            if joint.mimic is None:
                return joints[joint.name]
            return joint.mimic.multiplier * value(joint.mimic.joint) + joint.mimic.offset

        T = numpy.identity(4)
        joint = self.links[link]
        while joint is not None:
            T_offset = joint.T  # fixed transform from parent to joint frame
            # TODO: combine with joint's motion transform (rotation / translation along joint axis)
            if joint.jtype == Joint.revolute:
                T_motion = tf.quaternion_matrix(tf.quaternion_about_axis(angle=value(joint), axis=joint.axis))
            elif joint.jtype == Joint.prismatic:
                T_motion = tf.translation_matrix(value(joint) * joint.axis)
            elif joint.jtype == Joint.fixed:
                pass
            else:
                raise Exception("unknown joint type: " + str(joint.jtype))
            # TODO: actually compute forward kinematics
            joint = self.links[joint.parent]
        return T


# code executed when directly running this script
if __name__ == "__main__":
    import random
    from markers import frame, MarkerArray
    from sensor_msgs.msg import JointState

    rospy.init_node('test_node')
    pub = rospy.Publisher('/target_joint_states', JointState, queue_size=10)
    marker_pub = rospy.Publisher('/marker_array', MarkerArray, queue_size=10)

    robot = RobotModel()
    while not rospy.is_shutdown():
        joints = {j.name: random.uniform(j.min, j.max) for j in robot.active_joints}
        pub.publish(JointState(name=joints.keys(), position=joints.values()))

        T = robot.fk('panda_rightfinger', joints)
        marker_pub.publish(frame(T))

        rospy.rostime.wallsleep(1)
