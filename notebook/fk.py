#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from sensor_msgs.msg import JointState
from visualization_msgs.msg import MarkerArray

from markers import frame
from robot_model import RobotModel


class FKNode(Node):
    def __init__(self):
        super().__init__("fk")
        self.declare_parameter("target_link", "panda_link8")
        self.target_link = (
            self.get_parameter("target_link").get_parameter_value().string_value
        )

        self.robot = RobotModel(self)
        self.marker_pub = self.create_publisher(MarkerArray, "/marker_array", 1)
        self.sub = self.create_subscription(
            JointState, "/joint_states", self.publish_fk_marker, 10
        )

    def publish_fk_marker(self, msg: JointState):
        joints = {j: v for j, v in zip(msg.name, msg.position)}
        T, _ = self.robot.fk(link=self.target_link, joints=joints)
        self.marker_pub.publish(frame(T))


if __name__ == "__main__":
    rclpy.init()
    node = FKNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    else:
        rclpy.shutdown()
