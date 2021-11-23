import sys
import threading
import signal
import numpy
import rospy
from controller import Controller
from markers import iPoseMarker

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
    def __init__(self, controller):
        super(Gui, self).__init__()
        self.controller = controller
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
        c.addMarker(iPoseMarker(c.T))

        ns_old = numpy.zeros((c.N, 0))
        while not rospy.is_shutdown():
            # position + orientation control
            tasks = [c.pose_task(c.targets['pose'], c.T)]

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
    rospy.init_node('ik')
    app = QApplication(sys.argv)
    app.setApplicationDisplayName("Constrained-based Control Demo")
    gui = Gui(Controller())
    gui.show()
    threading.Thread(target=gui.loop).start()
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    sys.exit(app.exec_())
except rospy.ROSInterruptException:
    pass
