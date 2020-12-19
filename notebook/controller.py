#!/usr/bin/python

import numpy
import rospy
import random
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped, Transform, Pose, Quaternion, Vector3, Point
from tf import transformations as tf
from interactive_markers.interactive_marker_server import InteractiveMarkerServer, InteractiveMarkerFeedback
from robot_model import RobotModel, Joint
from markers import iPoseMarker


class Controller(object):
    damping = 0.1
    threshold = 0.1

    def __init__(self, pose=TransformStamped(header=Header(frame_id='panda_link8'), child_frame_id='target',
                                             transform=Transform(rotation=Quaternion(*tf.quaternion_about_axis(numpy.pi/4, [0, 0, 1])),
                                                                 translation=Vector3(0, 0, 0.105)))):

        self.robot = RobotModel()
        self.robot._add(Joint(pose))  # add a fixed end-effector transform
        self.pub = rospy.Publisher('/target_joint_states', JointState, queue_size=10)
        self.joint_msg = JointState()
        self.joint_msg.name = [j.name for j in self.robot.active_joints]
        self.reset()
        self.target_link = pose.child_frame_id
        self.T, self.J = self.robot.fk(self.target_link, dict(zip(self.joint_msg.name, self.joint_msg.position)))
        self.preferred_joints = self.joint_msg.position.copy()
        self.joint_weights = numpy.ones(len(self.joint_msg.position))
        self.cartesian_weights = numpy.ones(6)
        self.mins = numpy.array([j.min for j in self.robot.active_joints])
        self.maxs = numpy.array([j.max for j in self.robot.active_joints])
        self.prismatic = numpy.array([j.jtype == j.prismatic for j in self.robot.active_joints])

        self.targets = dict()
        self.im_server = InteractiveMarkerServer('controller')

    def addMarker(self, im):
        self.process_marker_feedback(InteractiveMarkerFeedback(marker_name=im.name, pose=im.pose))  # initialize target
        self.im_server.insert(im, self.process_marker_feedback)
        self.im_server.applyChanges()

    def process_marker_feedback(self, feedback, name=None):
        q = feedback.pose.orientation
        p = feedback.pose.position
        T = tf.quaternion_matrix(numpy.array([q.x, q.y, q.z, q.w]))
        T[0:3, 3] = numpy.array([p.x, p.y, p.z])
        self.targets[feedback.marker_name] = T

    def reset(self):
        self.joint_msg.position = numpy.asarray(
            [(j.min+j.max)/2 + 0.1*(j.max-j.min)*random.uniform(0, 1) for j in self.robot.active_joints])

    def actuate(self, q_delta):
        self.joint_msg.position += q_delta.ravel()
        # clip (prismatic) joints
        self.joint_msg.position[self.prismatic] = numpy.clip(self.joint_msg.position[self.prismatic],
                                                             self.mins[self.prismatic], self.maxs[self.prismatic])
        self.pub.publish(self.joint_msg)
        self.T, self.J = self.robot.fk(self.target_link, dict(zip(self.joint_msg.name, self.joint_msg.position)))

    def solve(self, tasks):
        def invert_clip(s):
            return 1./s if s > self.threshold else 0.

        def invert_damp(s):
            return s/(s**2 + self.damping**2)

        def invert_smooth_clip(s):
            return s/(self.threshold**2) if s < self.threshold else 1./s

        N = numpy.identity(self.J.shape[1])  # nullspace projector of previous tasks
        JA = numpy.zeros((0, N.shape[0]))  # accumulated Jacobians
        qdot = numpy.zeros(N.shape[0])

        for J, e in tasks:
            U, S, Vt = numpy.linalg.svd(J.dot(N) * self.joint_weights[None, :])
            # compute V'.T = V.T * Mq.T
            Vt *= self.joint_weights[None, :]

            rank = min(U.shape[0], Vt.shape[1])
            for i in range(rank):
                S[i] = invert_smooth_clip(S[i])

            qdot += numpy.dot(Vt.T[:, 0:rank], S * U.T.dot(numpy.array(e) - J.dot(qdot))).reshape(qdot.shape)

            # compute new nullspace projector
            JA = numpy.vstack([JA, J])
            U, S, Vt = numpy.linalg.svd(JA)
            accepted_singular_values = (S > 1e-3).sum()
            VN = Vt[accepted_singular_values:].T
            N = VN.dot(VN.T)
        self.nullspace = VN  # remember nullspace basis
        return qdot

    @staticmethod
    def stack(tasks):
        """Combine all tasks by stacking them into a single Jacobian"""
        Js, errs = zip(*tasks)
        return numpy.vstack(Js), numpy.hstack(errs)

    def position_task(self, T_tgt, T_cur, scale=1.0):
        """Move eef towards a specific target point in base frame"""
        return self.J[:3], scale*(T_tgt[0:3, 3]-T_cur[0:3, 3])

    def orientation_task(self, T_tgt, T_cur, scale=1.0):
        """Move eef into a specific target orientation in base frame"""
        delta = numpy.identity(4)
        delta[0:3, 0:3] = T_cur[0:3, 0:3].T.dot(T_tgt[0:3, 0:3])
        angle, axis, _ = tf.rotation_from_matrix(delta)
        # transform rotational velocity from end-effector into base frame orientation (only R!)
        return self.J[3:], scale*(T_cur[0:3, 0:3].dot(angle * axis))

    def pose_task(self, T_tgt, T_cur, scale=(1.0, 1.0)):
        """Perform position and orientation task with same priority"""
        return self.stack([self.position_task(T_tgt, T_cur, scale=scale[0]),
                           self.orientation_task(T_tgt, T_cur, scale=scale[1])])

    def joint_task(self, scale=0.1):
        """Move towards a set of preferred joints, consumes all DOFs"""
        qerr = self.joint_msg.position - self.preferred_joints
        J = numpy.identity(self.J.shape[1])
        return J, -scale*qerr

    def position_control(self, target):
        q_delta = self.solve(self.position_task(target, self.T))
        self.actuate(q_delta)

    def pose_control(self, target):
        q_delta = self.solve(self.pose_task(target, self.T))
        self.actuate(q_delta)

    def hierarchic_control(self, target):
        # position control > orientation control > joints
        q_delta = self.solve([self.position_task(target, self.T),
                              self.orientation_task(target, self.T),
                              self.joint_task()])
        # pose > joints
        #q_delta = self.solve([self.pose_task(target, self.T), self.joint_task()])
        self.actuate(q_delta)

    def lissajous(self, w=0.1*2*numpy.pi, n=2):
        # Compute offset for Lissajous figure
        t = rospy.get_time()
        offset = numpy.asarray([0.3 * numpy.sin(w * t), 0.3 * numpy.sin(n * w * t), 0.])
        # add offset to current marker pose to draw Lissajous figure in x-y-plane of marker
        target = numpy.copy(self.targets['pose'])
        target[0:3, 3] += target[0:3, 0:3].dot(offset)
        self.pose_control(target)


if __name__ == '__main__':
    rospy.init_node('ik')
    c = Controller()
    c.addMarker(iPoseMarker(c.T))
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        c.hierarchic_control(c.targets['pose'])
        rate.sleep()
