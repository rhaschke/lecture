#!/usr/bin/python

import numpy
import qpsolvers
import rospy
import random
from std_msgs.msg import Header, ColorRGBA
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped, Transform, Pose, Quaternion, Vector3, Point
from visualization_msgs.msg import Marker, MarkerArray
from tf import transformations as tf
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from robot_model import RobotModel, Joint
from markers import addMarker, processFeedback, iPoseMarker, frame


def skew(w):
    return numpy.array([[0, -w[2], w[1]],
                        [w[2], 0, -w[0]],
                        [-w[1], w[0], 0]])


class Controller(object):
    damping = 0.1
    threshold = 1.0

    def __init__(self, pose=TransformStamped(header=Header(frame_id='panda_link8'), child_frame_id='target',
                                             transform=Transform(rotation=Quaternion(*tf.quaternion_about_axis(numpy.pi/4, [0, 0, 1])),
                                                                 translation=Vector3(0, 0, 0.105)))):

        self.robot = RobotModel()
        self.robot._add(Joint(pose))  # add a fixed end-effector transform
        self.joint_pub = rospy.Publisher('/target_joint_states', JointState, queue_size=10)
        self.joint_msg = JointState()
        self.joint_msg.name = [j.name for j in self.robot.active_joints]
        self.reset()
        self.target_link = pose.child_frame_id
        self.T, self.J = self.robot.fk(self.target_link, dict(zip(self.joint_msg.name, self.joint_msg.position)))
        self.N = self.J.shape[1]  # number of (active) joints
        self.preferred_joints = self.joint_msg.position.copy()
        self.joint_weights = numpy.ones(self.N)
        self.cartesian_weights = numpy.ones(6)
        self.mins = numpy.array([j.min for j in self.robot.active_joints])
        self.maxs = numpy.array([j.max for j in self.robot.active_joints])
        self.prismatic = numpy.array([j.jtype == j.prismatic for j in self.robot.active_joints])

        # prepare publishing eef trace
        self.trace_marker = Marker(type=Marker.LINE_STRIP, header=Header(frame_id='world'),
                                   ns='trace', color=ColorRGBA(0, 1, 1, 0.5))
        self.trace_marker.pose.orientation.w = 1
        self.trace_marker.scale.x = 0.01  # line width
        self.marker_pub = rospy.Publisher('/marker_array', MarkerArray, queue_size=10)

        self.targets = dict()

    def setTarget(self, name, goal):
        self.targets[name] = goal

    def reset(self):
        self.joint_msg.position = numpy.asarray(
            [(j.min+j.max)/2 + 0.1*(j.max-j.min)*random.uniform(0, 1) for j in self.robot.active_joints])

    def actuate(self, q_delta):
        """Move robot by given changes to joint angles"""
        self.joint_msg.position += q_delta.ravel()  # add (numpy) vector q_delta to current joint position vector
        # clip (prismatic) joints
        self.joint_msg.position[self.prismatic] = numpy.clip(self.joint_msg.position[self.prismatic],
                                                             self.mins[self.prismatic], self.maxs[self.prismatic])
        self.joint_pub.publish(self.joint_msg)  # publish new joint state
        joints = dict(zip(self.joint_msg.name, self.joint_msg.position))  # turn list of names and joint values into map
        self.T, self.J = self.robot.fk(self.target_link, joints)  # compute new forward kinematics and Jacobian

        # publish eef marker
        msg = MarkerArray(markers=frame(self.T, scale=0.05, ns='eef frame'))
        trace = self.trace_marker.points
        trace.append(Point(*self.T[0:3, 3]))
        if (len(trace) > 1000):
            del trace[0]
        msg.markers.append(self.trace_marker)
        self.marker_pub.publish(msg)

    def solve(self, tasks):
        """Hierarchically solve tasks of the form J dq = e"""
        def invert_clip(s):
            return 1./s if s > self.threshold else 0.

        def invert_damp(s):
            return s/(s**2 + self.damping**2)

        def invert_smooth_clip(s):
            return s/(self.threshold**2) if s < self.threshold else 1./s

        N = numpy.identity(self.N)  # nullspace projector of previous tasks
        JA = numpy.zeros((0, self.N))  # accumulated Jacobians
        qdot = numpy.zeros(self.N)

        if isinstance(tasks, tuple):
            tasks = [tasks]

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

    def solve_qp(self, tasks):
        """Solve tasks (J, ub, lb) of the form lb ≤ J dq ≤ ub
           using quadratic optimization: https://pypi.org/project/qpsolvers"""
        maxM = numpy.amax([task[0].shape[0] for task in tasks]) # max task dimension
        sumM = numpy.sum([task[0].shape[0] for task in tasks]) # sum of all task dimensions
        usedM = 0
        # allocate arrays once
        G, h = numpy.zeros((2*sumM, self.N + maxM)), numpy.zeros(2*sumM)
        P = numpy.identity(self.N+maxM)
        P[self.N:, self.N:] *= 1.0  # use different scaling for slack variables?
        q = numpy.zeros(self.N + maxM)

        # joint velocity bounds + slack bounds
        upper = numpy.hstack([numpy.minimum(0.1, self.maxs - self.joint_msg.position), numpy.zeros(maxM)])
        lower = numpy.hstack([numpy.maximum(-0.1, self.mins - self.joint_msg.position), numpy.full(maxM, -numpy.infty)])

        # fallback solution
        dq = numpy.zeros(self.N)

        def add_constraint(A, bound):
            G[usedM:usedM+M, :N] = A
            G[usedM:usedM+M, N:N+M] = numpy.identity(M)  # allow (negative) slack variables
            h[usedM:usedM+M] = bound
            return usedM + M

        for idx, task in enumerate(tasks):
            try:  # inequality tasks are pairs of (J, ub, lb=None)
                J, ub, lb = task
            except ValueError:  # equality tasks are pairs of (J, err)
                J, ub = task
                lb = ub  # turn into inequality task: err ≤ J dq ≤ err
            J = numpy.atleast_2d(J)
            M, N = J.shape

            # augment G, h with current task's constraints
            oldM = usedM
            usedM = add_constraint(J, ub)
            if lb is not None:
                usedM = add_constraint(-J, -lb)

            result = qpsolvers.solve_qp(P=P[:N+M, :N+M], q=q[:N+M],
                                        G=G[:usedM, :N+M], h=h[:usedM], A=None, b=None,
                                        lb=lower[:N+M], ub=upper[:N+M],
                                        solver='osqp')
            if result is None:
                print("{}: failed  ".format(idx), end='')
                usedM = oldM  # ignore subtask and continue with subsequent tasks
            else: # adapt added constraints for next iteration
                dq, slacks = result[:N], result[N:]
                print("{}:".format(idx), slacks, " ", end='')
                G[oldM:usedM,N:N+M] = 0
                h[oldM:oldM+M] += slacks
                if oldM+M < usedM:
                    h[oldM+M:usedM] -= slacks
        print()
        self.nullspace = numpy.zeros((self.N, 0))
        return dq

    @staticmethod
    def vstack(items):
        return numpy.vstack(items) if items else None

    @staticmethod
    def hstack(items):
        return numpy.hstack(items) if items else None

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

    def distance_task(self, T_tgt, T_cur, dist=0, scale=1.0):
        """Keep distance to target position, not considering (approach) direction"""
        delta = T_cur[0:3, 3] - T_tgt[0:3, 3]
        return delta.T.dot(self.J[:3]), -scale * (numpy.linalg.norm(delta) - dist)

    def plane_task(self, normal, dist, scale=1.0):
        """Move eef within plane given by normal vector and distance to origin"""
        return normal.T.dot(self.J[:3]), -scale * (normal.dot(self.T[0:3, 3]) - dist)

    def parallel_axes_task(self, axis, reference, scale=1.0):
        """Align axis in eef frame to be parallel to reference axis in base frame"""
        axis = self.T[0:3, 0:3].dot(axis)  # transform axis from eef frame to base frame
        return (skew(reference).dot(skew(axis))).dot(self.J[3:]), scale * numpy.cross(reference, axis)

    def cone_task(self, axis, reference, threshold):
        """Align axis in eef frame to lie in cone spanned by reference axis and opening angle acos(threshold)"""
        axis = self.T[0:3, 0:3].dot(axis)  # transform axis from eef frame to base frame
        return reference.T.dot(skew(axis)).dot(self.J[3:]), (reference.T.dot(axis) - threshold), None

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
    rospy.init_node('ik')  # create a ROS node
    c = Controller()

    ims = InteractiveMarkerServer('controller')
    addMarker(ims, iPoseMarker(c.T), processFeedback(c.setTarget))
    ims.applyChanges()

    rate = rospy.Rate(50)  # Run control loop at 50 Hz
    while not rospy.is_shutdown():
        c.hierarchic_control(c.targets['pose'])
        rate.sleep()
