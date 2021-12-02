import sys
import threading
import signal
import numpy
import rospy
from argparse import ArgumentParser
from controller import Controller
from robot_model import adjoint
from markers import iPositionMarker, iPoseMarker, iPlaneMarker, iConeMarker, addMarker, processFeedback, sphere, box, plane, frame
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from geometry_msgs.msg import Vector3
from tf import transformations as tf
from visualization_msgs.msg import MarkerArray

from python_qt_binding.QtCore import Qt
from python_qt_binding.QtGui import QFontDatabase
from python_qt_binding.QtWidgets import QApplication, QWidget, QSlider, QLabel, QVBoxLayout


class Slider(QSlider):
    """QSlider variant that jumps back to zero when released"""

    def __init__(self):
        super(Slider, self).__init__(Qt.Horizontal)
        self.setRange(-1000, 1000)
        self.setValue(0)
        self.sliderReleased.connect(lambda: self.setValue(0))


class Gui(QWidget):
    def __init__(self, controller, task_id):
        super(Gui, self).__init__()
        self.controller = controller
        self.task_id = task_id
        self.errors = QLabel()
        self.errors.setFont(QFontDatabase.systemFont(QFontDatabase.FixedFont))
        self.sliders = []

        layout = QVBoxLayout()
        layout.addWidget(QLabel('errors:'))
        layout.addWidget(self.errors)
        layout.addWidget(QLabel('nullspace control:'))
        for _ in range(controller.N):
            slider = Slider()
            self.sliders.append(slider)
            layout.addWidget(slider)
        self.setLayout(layout)

    def onValueChanged(self, index, value):
        self.ns[index] = value

    def showErrors(self, tasks):
        numpy.set_printoptions(precision=3, suppress=True)
        self.errors.setText('\n'.join([str(e) for J, e in tasks]))

    def loop(self):
        rate = rospy.Rate(50)
        c = self.controller
        ims = InteractiveMarkerServer('controller')
        task = self.task_id
        if task == 0:  # pose task
            addMarker(ims, iPoseMarker(c.T), processFeedback(c.setTarget))
        elif task >= 1 and task <= 4:  # move in plane
            markers = [plane()]
            if task != 1:
                markers.append(sphere())
            addMarker(ims, iPlaneMarker(c.T[0:3, 3], markers=markers),
                      processFeedback(c.setTarget))
        elif task >= 5 and task <= 6:  # constrain position to box and orientation to a cone
            tol = 0.5 * numpy.array([0.2, 0.1, 0.05])  # tolerance box
            addMarker(ims, iPositionMarker(c.T[0:3, 3],
                                           markers=[sphere(), box(size=Vector3(*(2.*tol)))]),
                      processFeedback(c.setTarget))
        if task in [4, 6]:  # constrain orientation to cone
            eef_axis = numpy.array([0, 1, 0])  # y-axis to align with cone axis
            T = c.T
            T[0:3, 3] = numpy.array([-0.25, 0, 0])  # place cone next to the robot
            # add cone marker aligned with current eef's y-axis
            cm = iConeMarker(ims, T.dot(tf.rotation_matrix(numpy.pi/2, [-1, 0, 0])),
                             pose_cb=c.setTarget, angle_cb=c.setTarget)
        ims.applyChanges()

        ns_old = numpy.zeros((c.N, 0))
        while not rospy.is_shutdown():
            if task == 0:  # position + orientation control
                tasks = [c.pose_task(c.targets['pose'], c.T)]
            elif task >= 1 and task <= 4:
                # move on a plane spanned by xy axes of marker
                T = c.targets['plane']
                normal = T[0:3, 2]  # plane normal from marker's z-axis
                tasks = [c.plane_task(normal, normal.dot(T[0:3, 3]), scale=0.1)]
                if task >= 2:
                    tasks.append(c.distance_task(T, c.T, dist=0.2, scale=0.1))  # maintain distance 0.2 to center
                    # tasks.append((normal.T.dot(c.J[3:]), 0.01))  # rotates with constant speed about normal axis
                    # rotate about center with linear velocity: w x r
                    r = T[0:3, 3] - c.T[0:3, 3]
                    tasks.append((c.J[:3], numpy.cross(0.01*normal, r)))

                # keep gripper aligned with normal axis
                normal = numpy.array([0, 0, 1])  # world z-axis
                if task == 3:  # using parallel_axes_task
                    tasks.append(c.parallel_axes_task(numpy.array([0, 1, 0]), normal))
            elif task >= 5 and task <= 6:  # constrain position
                J, e = c.position_task(c.targets['pos'], c.T)
                lb_violated, ub_violated = (e < -tol), (e > tol)
                violated = lb_violated | ub_violated
                # clip errors to box boundaries
                clipped = numpy.array(e, copy=True)
                clipped[lb_violated] -= -tol[lb_violated]
                clipped[ub_violated] -= tol[ub_violated]
                # if error violates box constraint, move into box via equality task and clipped error
                tasks = [(J[violated], clipped[violated])]
            if task in [4, 6]:  # additionally constrain orientation to cone
                cone_pose = c.targets['cone_pose']
                angle = c.targets['cone_angle']
                tasks.append(c.cone_task(eef_axis, cone_pose[0:3, 2], threshold=numpy.cos(angle)))

                # publish orientation of eef_axis as 2nd cylinder of eef frame
                T = c.T
                T[:3, 3] = cone_pose[:3, 3]
                c.marker_pub.publish(MarkerArray(markers=frame(T, scale=0.2, radius=0.01, ns='eef orientation')[1:2]))

            self.showErrors(tasks)
            q_delta = c.solve(tasks)

            # nullspace control
            N = c.nullspace.shape[1]  # dimensionality of nullspace
            ns = numpy.array([s.value() for s in self.sliders[:N]])  # slider values
            # disable inactive nullspace sliders
            [s.setEnabled(i < N) for i, s in enumerate(self.sliders)]
            # align new nullspace basis to previous one (vectors might be flipped)
            N = min(N, ns_old.shape[1])  # minimum number of columns before and now
            c.nullspace[:, :N] *= numpy.sign(numpy.sum(c.nullspace[:, :N] * ns_old[:, :N], axis=0))
            ns_old = c.nullspace
            # apply nullspace motion
            q_delta += c.nullspace.dot(1e-4 * ns)

            c.actuate(q_delta)
            rate.sleep()


try:
    parser = ArgumentParser()
    parser.add_argument("mode", type=int, nargs='?', default=0, help="control mode")
    args = parser.parse_args()

    rospy.init_node('ik')
    app = QApplication(sys.argv)
    app.setApplicationDisplayName("Constrained-based Control Demo")
    gui = Gui(Controller(), args.mode)
    gui.show()
    threading.Thread(target=gui.loop).start()
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    sys.exit(app.exec_())
except rospy.ROSInterruptException:
    pass
