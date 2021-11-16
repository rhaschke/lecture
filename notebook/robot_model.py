# python 2/3 compatibility: interpret print() as a function
from __future__ import print_function

import numpy
import rospy
import xml.dom.minidom
from tf import transformations as tf


def get_value(xml, child=None, attribute=None):
    """Get value of given attribute. If child arg is present, fetch first child of given name."""
    if child is not None:
        xml = xml.getElementsByTagName(child)[0]
    if attribute is not None:
        return xml.getAttribute(attribute)


def parse_vector(s):
    """Parse a string of shape "0.1 0.2 0.3" into a vector of floats"""
    return numpy.array([float(v) for v in s.split(' ')])


def hat(p):
    """Return skew-symmetric matrix for given vector p"""
    return numpy.array([[0, -p[2], p[1]],
                        [p[2], 0, -p[0]],
                        [-p[1], p[0], 0]])


def adjoint(T, inverse=False):
    """Return adjoint matrix for homogenous transform T (or its inverse)."""
    if T.shape == (4, 4): # input T is homogenous transform
        R = T[0:3, 0:3]
        p = T[0:3, 3]
    elif T.shape == (3, 3): # input T is rotation matrix
        R = T
        p = numpy.zeros(3)
    else: # input T is position vector
        R = numpy.identity(3)
        p = T
    if not inverse:
        return numpy.block([[R, hat(p).dot(R)], [numpy.zeros((3, 3)), R]])
    else:
        return numpy.block([[R.T, R.T.dot(hat(-p))], [numpy.zeros((3, 3)), R.T]])


class Mimic():
    """Load (and represent) the <mimic> tag of an URDF joint"""
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
    """Class representing a single URDF joint"""
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
        self.jtype = getattr(Joint, tag.getAttribute('type'))  # map joint-type string onto enum
        self.active = self.jtype in [Joint.revolute, Joint.prismatic]  # is the joint considered active?
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
    """Class representing the kinematic tree of a robot"""
    def __init__(self, param='robot_description'):
        self.links = {}  # map link names to its parent joints
        self.joints = {}  # map joint names to joint instances
        self.active_joints = []  # list of active, non-mimic joints

        description = rospy.get_param(param)  # fetch URDF from ROS parameter server
        doc = xml.dom.minidom.parseString(description)  # parse URDF string into dom
        robot = doc.getElementsByTagName('robot')[0]
        for tag in robot.getElementsByTagName('joint'):  # process all <joint> tags
            self._add(Joint(tag))  # parse and add the joint to the kinematic tree

    def _add(self, joint):
        """Add a single joint to the kinematic tree"""
        self.joints[joint.name] = joint
        if joint.active and joint.mimic is None:
            self.active_joints.append(joint)
        if joint.mimic is not None:
            joint.mimic.joint = self.joints[joint.mimic.joint]  # replace name with instance
        self.links[joint.child] = joint
        if joint.parent not in self.links:
            self.links[joint.parent] = None

    def fk(self, link, joints):
        """Compute forward kinematics up to given link using given map of joint angles"""
        def value(joint):
            """Get joint value from joints, considering mimic joints"""
            if joint.mimic is None:
                return joints[joint.name]
            return joint.mimic.multiplier * value(joint.mimic.joint) + joint.mimic.offset

        def index(joint):
            """Get joint index (into self.active_joint) and the velocity scaling factor"""
            if joint.mimic is None:
                return next((i for i, j in enumerate(self.active_joints) if j is joint), None), 1.0
            idx, scale = index(joint.mimic.joint)
            return idx, joint.mimic.multiplier * scale

        T = numpy.identity(4)
        J = numpy.zeros((6, len(self.active_joints)))
        joint = self.links[link]
        while joint is not None:
            T_offset = joint.T  # fixed transform from parent to joint frame
            # post-multiply joint's motion transform (rotation / translation along joint axis)
            if joint.jtype == Joint.revolute:
                # transform twist from current joint frame (joint.axis) into eef frame (via T^-1)
                twist = adjoint(T, inverse=True).dot(numpy.block([numpy.zeros(3), joint.axis]))
                T_motion = tf.quaternion_matrix(tf.quaternion_about_axis(angle=value(joint), axis=joint.axis))
                T_offset = T_offset.dot(T_motion)
            elif joint.jtype == Joint.prismatic:
                twist = adjoint(T, inverse=True).dot(numpy.block([joint.axis, numpy.zeros(3)]))
                T_motion = tf.translation_matrix(value(joint) * joint.axis)
                T_offset = T_offset.dot(T_motion)
            elif joint.jtype == Joint.fixed:
                pass  # no action: fixed frames don't move
            else:
                raise Exception("unknown joint type: " + str(joint.jtype))
            # pre-multiply joint transform with T (because traversing from eef to root)
            T = T_offset.dot(T)  # T' = joint.T * T_motion * T

            # update the Jacobian
            idx, scale = index(joint)  # find active joint index for given joint
            if idx is not None:  # ignore fixed joints
                J[:, idx] += scale * twist  # add twist contribution, optionally scaled by mimic joint's multiplier

            # climb upwards to parent joint
            joint = self.links[joint.parent]

        # As we climbed the kinematic tree from end-effector to base frame, we have
        # represented all Jacobian twists w.r.t. the end-effector frame.
        # Now transform all twists into the orientation of the base frame
        R = T[0:3, 0:3]
        Ad_R = numpy.block([[R, numpy.zeros((3, 3))], [numpy.zeros((3, 3)), R]])
        return T, Ad_R.dot(J)


# testing code executed when directly running this script
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

        T, J = robot.fk('panda_link8', joints)
        marker_pub.publish(frame(T))

        rospy.rostime.wallsleep(1)
