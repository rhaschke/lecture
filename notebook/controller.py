#!/usr/bin/env python3

import random
import numpy
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped, Transform, Vector3
from visualization_msgs.msg import Marker, MarkerArray
import tf_transformations as tf
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from robot_model import RobotModel, Joint
from markers import addMarker, processFeedback, iPoseMarker, frame, point, quaternion, colorRGBA

class Controller(object):
    damping = 0.1
    threshold = 0.1

    def __init__(
        self,
        node,
        pose=TransformStamped(
            header=Header(frame_id="panda_link8"),
            child_frame_id="target",
            transform=Transform(
                rotation=quaternion(*tf.quaternion_about_axis(numpy.pi / 4, [0, 0, 1])),
                translation=Vector3(x=0.0, y=0.0, z=0.105),
            ),
        ),
    ):

        self.node = node
        self.robot = RobotModel(node)
        self.robot._add(Joint(pose))  # add a fixed end-effector transform
        self.joint_pub = self.node.create_publisher(JointState, "/target_joint_states", 10)
        self.joint_msg = JointState()
        self.joint_msg.name = [j.name for j in self.robot.active_joints]
        self.reset()
        self.target_link = pose.child_frame_id
        self.T, self.J = self.robot.fk(self.target_link, dict(zip(self.joint_msg.name, self.joint_msg.position)))
        self.preferred_joints = numpy.array(self.joint_msg.position, copy=True)
        self.joint_weights = numpy.ones(len(self.joint_msg.position))
        self.cartesian_weights = numpy.ones(6)

        # prepare publishing eef trace
        self.trace_marker = Marker(type=Marker.LINE_STRIP, header=Header(frame_id='world'),
                                   ns='trace', color=colorRGBA(0, 1, 1, 0.5))
        self.trace_marker.pose.orientation.w = 1.
        self.trace_marker.scale.x = 0.01  # line width
        self.marker_pub = self.node.create_publisher(MarkerArray, '/marker_array', 10)

        self.targets = dict()

    def setTarget(self, name, goal):
        self.targets[name] = goal

    def reset(self):
        self.joint_msg.position = [
            (j.min + j.max) / 2 + 0.1 * (j.max - j.min) * random.uniform(0, 1)
            for j in self.robot.active_joints
        ]

    def actuate(self, q_delta):
        """Move robot by given changes to joint angles"""
        self.joint_msg.position += q_delta.ravel()
        self.joint_pub.publish(self.joint_msg)  # publish new joint state
        joints = dict(zip(self.joint_msg.name, self.joint_msg.position))  # turn list of names and joint values into map
        self.T, self.J = self.robot.fk(self.target_link, joints)  # compute new forward kinematics and Jacobian

        # publish eef marker
        msg = MarkerArray(markers=frame(self.T, scale=0.05, ns='eef frame'))
        trace = self.trace_marker.points
        trace.append(point(*self.T[0:3, 3]))
        if (len(trace) > 1000):
            del trace[0]
        msg.markers.append(self.trace_marker)
        self.marker_pub.publish(msg)

    def solve(self, J, error):
        """Inverse velocity kinematics: q_delta = J^+ * error"""
        def invert_clip(s):
            return 1./s if s > self.threshold else 0.

        def invert_damp(s):
            return s / (s**2 + self.damping**2)

        def invert_smooth_clip(s):
            return 1./s if s > self.threshold else s / (self.threshold**2)

        def invert_smooth_damped(s):
            return 1./s if s > self.threshold else s / (s**2 + (s - self.threshold)**2)

        # SVD of Mx * J * Mq
        U, S, Vt = numpy.linalg.svd(self.cartesian_weights[:, None] * J * self.joint_weights[None, :])
        # compute V'.T = V.T * Mq.T
        Vt *= self.joint_weights[None, :]

        rank = min(U.shape[0], Vt.shape[1])
        accepted_singular_values = (S > 1e-3).sum()
        VN = Vt[accepted_singular_values:].T
        for i in range(rank):
            S[i] = invert_smooth_clip(S[i])

        qdot1 = numpy.dot(Vt.T[:, 0:rank], S * U.T.dot(self.cartesian_weights * numpy.array(error)))
        qdot2 = VN.dot(0.1 * VN.T.dot(self.preferred_joints - self.joint_msg.position))
        return qdot1 + qdot2

    @staticmethod
    def position_error(T_tgt, T_cur):
        """Given homogenous transforms of target and current pose, compute error vector"""
        return T_tgt[0:3, 3]-T_cur[0:3, 3]

    @staticmethod
    def orientation_error(T_tgt, T_cur):
        delta = numpy.identity(4)
        delta[0:3, 0:3] = T_cur[0:3, 0:3].T.dot(T_tgt[0:3, 0:3])
        angle, axis, _ = tf.rotation_from_matrix(delta)
        # transform rotational velocity from end-effector into base frame orientation (only R!)
        return T_cur[0:3, 0:3].dot(angle * axis)

    def position_control(self, target):
        v = self.position_error(target, self.T)
        q_delta = self.solve(self.J[0:3, :], v)
        self.actuate(q_delta)

    def pose_control(self, target):
        v = self.position_error(target, self.T)
        w = self.orientation_error(target, self.T)
        q_delta = self.solve(self.J, numpy.block([0.1*v, 0.1*w]))
        self.actuate(q_delta)

    def lissajous(self, w=0.1*2*numpy.pi, n=2):
        # Compute offset for Lissajous figure
        t = self.node.get_clock().now().nanoseconds * 1e-9
        offset = numpy.asarray([0.3 * numpy.sin(w * t), 0.3 * numpy.sin(n * w * t), 0.])
        # add offset to current marker pose to draw Lissajous figure in x-y-plane of marker
        target = numpy.copy(self.targets["pose"])
        target[0:3, 3] += target[0:3, 0:3].dot(offset)
        self.position_control(target)


if __name__ == '__main__':
    rclpy.init()
    node = Node("ik")

    try:
        c = Controller(node)

        ims = InteractiveMarkerServer(node, "controller")
        addMarker(ims, iPoseMarker(c.T), processFeedback(c.setTarget))
        ims.applyChanges()

        timer = node.create_timer(0.02, lambda: c.pose_control(c.targets["pose"]))
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    else:
        node.destroy_node()
        rclpy.shutdown()
