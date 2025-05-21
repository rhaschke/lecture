# Import rospy and initialize a python ROS node
import rospy
rospy.init_node('mynode')

# Import and instantiate the robot model
from robot_model import RobotModel
robot = RobotModel()

# Define a callback function to compute the forward-kinematics and publish a frame marker
from sensor_msgs.msg import JointState
from visualization_msgs.msg import MarkerArray
from markers import frame
marker_pub = rospy.Publisher('/marker_array', MarkerArray, queue_size=1)


def publish_fk_marker(msg: JointState):
    T, _ = robot.fk(link='panda_link8', joints={j: v for j, v in zip(msg.name, msg.position)})
    marker_pub.publish(frame(T))


# Subscribe to the /joint_states topic
sub = rospy.Subscriber('/joint_states', JointState, publish_fk_marker)

# Run ROS loop
rospy.spin()
