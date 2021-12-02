import sys
import threading
import signal
import numpy
import rospy
from argparse import ArgumentParser
from controller import Controller
from robot_model import adjoint
from markers import iPositionMarker, iPoseMarker, iPlaneMarker, addMarker, processFeedback, sphere, box, plane
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from geometry_msgs.msg import Vector3

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

    def __init__(self, controller, mode):
        super(Gui, self).__init__()
        self.controller = controller
        self.mode = mode
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
        mode = self.mode
        if "pose" in mode:
            addMarker(ims, iPoseMarker(c.T), processFeedback(c.setTarget))
        elif "plane" in mode:
            markers = [plane(), sphere()]
            addMarker(ims, iPlaneMarker(c.T[0:3, 3], markers=markers),
                      processFeedback(c.setTarget))
        elif "box" in mode:  # constrain position to box
            tol = 0.5 * numpy.array([0.2, 0.1, 0.05])  # tolerance box
            addMarker(ims, iPositionMarker(c.T[0:3, 3],
                                           markers=[sphere(), box(size=Vector3(*(2.*tol)))]),
                      processFeedback(c.setTarget))
        ims.applyChanges()

        ns_old = numpy.zeros((c.N, 0))
        while not rospy.is_shutdown():
            if "pose" in mode:  # position + orientation control
                tasks = [c.pose_task(c.targets['pose'], c.T)]

            elif "plane" in mode:
                # move on a plane spanned by xy axes of marker
                T = c.targets['plane']
                normal = T[0:3, 2]  # plane normal from marker's z-axis
                tasks = [c.plane_task(normal, normal.dot(T[0:3, 3]), scale=0.1)]

                if "dist" in mode:
                    tasks.append(c.distance_task(T, c.T, dist=0.2, scale=0.1))  # maintain distance 0.2 to center

                if "rotate" in mode:
                    tasks.append(
                        (normal.T.dot(c.J[3:]), 0.01)
                    )  # rotates with constant speed about normal axis
                elif "rotateCenter" in mode:
                    # rotate about center with linear velocity: w x r
                    r = T[0:3, 3] - c.T[0:3, 3]
                    tasks.append((c.J[:3], numpy.cross(0.1 * normal, r)))

            elif "box" in mode:  # constrain position
                J, e = c.position_task(c.targets['pos'], c.T)
                lb_violated, ub_violated = (e < -tol), (e > tol)
                violated = lb_violated | ub_violated
                # clip errors to box boundaries
                clipped = numpy.array(e, copy=True)
                clipped[lb_violated] -= -tol[lb_violated]
                clipped[ub_violated] -= tol[ub_violated]
                # if error violates box constraint, move into box via equality task and clipped error
                tasks = [(J[violated], clipped[violated])]

            # keep gripper aligned with normal axis
            normal = numpy.array([0, 0, 1])  # world z-axis
            if "parallel" in mode:  # using parallel_axes_task
                tasks.append(c.parallel_axes_task(numpy.array([0, 1, 0]), normal))
            elif "cone" in mode:  # using cone_task
                tasks.append(c.cone_task(numpy.array([0, 1, 0]), normal, threshold=1.0))

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
    parser.add_argument(
        "mode",
        type=str,
        nargs="*",
        default="pose",
        help="control mode",
        choices=[
            "pose",
            "plane",
            "box",
            "dist",
            "rotate",
            "rotateCenter",
            "parallel",
            "cone",
        ],
    )
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
