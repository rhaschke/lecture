/* ============================================================
 *
 * Copyright (C) 2015 by Robert Haschke <rhaschke at techfak dot uni-bielefeld dot de>
 *
 * This file may be licensed under the terms of the BSD license.
 *
 * The development of this software was supported by:
 *   CITEC, "Cognitive Interaction Technology" Excellence Cluster
 *   Bielefeld University, Germany
 *
 * ============================================================ */

#include <string>
#include <kdl/kdl.hpp>
#include <kdl/jntarray.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <angles/angles.h>
#include <Eigen/Eigen>
#include <interactive_markers/interactive_marker_server.hpp>
#include <tf2_kdl/tf2_kdl.hpp>

std::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
visualization_msgs::msg::InteractiveMarkerFeedback marker_feedback;
rclcpp::Node::SharedPtr node;

sensor_msgs::msg::JointState init_message(const KDL::Chain &chain)
{
	sensor_msgs::msg::JointState msg;
	for (unsigned int i=0; i < chain.getNrOfSegments(); ++i) {
		KDL::Segment segment = chain.getSegment(i);
		KDL::Joint joint = segment.getJoint();
		if (joint.getType() == KDL::Joint::None) continue;
		msg.name.push_back(joint.getName());
		msg.position.push_back(0);
	}
	return msg;
}

void update_message(sensor_msgs::msg::JointState &msg,
						  const KDL::JntArray &joints)
{
	msg.header.stamp = node->now();
	Eigen::Map<Eigen::VectorXd>(msg.position.data(), msg.position.size()) = joints.data;
}

void processFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback)
{
	marker_feedback = *feedback;
}

void operator<<(geometry_msgs::msg::Point &pos, const Eigen::Vector3d &p)
{
	pos.x = p[0];
	pos.y = p[1];
	pos.z = p[2];
}
void operator<<(geometry_msgs::msg::Quaternion &quat, const Eigen::Quaterniond &q)
{
	quat.w = q.w();
	quat.x = q.x();
	quat.y = q.y();
	quat.z = q.z();
}

visualization_msgs::msg::InteractiveMarker
createInteractiveMarker(const std::string &name,
								const geometry_msgs::msg::PoseStamped &stamped)
{
	visualization_msgs::msg::InteractiveMarker imarker;
	imarker.name = name;
	imarker.header = stamped.header;
	imarker.pose = stamped.pose;
	return imarker;
}
visualization_msgs::msg::Marker
createSphereMarker(double radius)
{
	visualization_msgs::msg::Marker marker;

	marker.type = visualization_msgs::msg::Marker::SPHERE;
	marker.scale.x = radius * 2.0;
	marker.scale.y = radius * 2.0;
	marker.scale.z = radius * 2.0;

	return marker;
}
visualization_msgs::msg::InteractiveMarkerControl
createViewPlaneControl(bool position, bool orientation)
{
	visualization_msgs::msg::InteractiveMarkerControl control;
	control.orientation_mode = visualization_msgs::msg::InteractiveMarkerControl::VIEW_FACING;

	if (position && orientation) {
		control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_ROTATE_3D;
	} else if (orientation) {
		control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_3D;
	} else
		control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_3D;

	control.independent_marker_orientation = true;
	control.always_visible = false;
	control.name = "move";

	return control;
}
void addPositionControl(visualization_msgs::msg::InteractiveMarker &imarker,
								const Eigen::Vector3d &axis, bool orientation_fixed)
{
	visualization_msgs::msg::InteractiveMarkerControl control;
	if (orientation_fixed)
		control.orientation_mode = visualization_msgs::msg::InteractiveMarkerControl::FIXED;
	control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;

	control.orientation << Eigen::Quaterniond::FromTwoVectors(
	                          Eigen::Vector3d::UnitX(), axis);
	imarker.controls.push_back(control);
}
void addPositionControls(visualization_msgs::msg::InteractiveMarker &imarker,
								 unsigned int axes, bool orientation_fixed = false)
{
	for (unsigned int i=0; i < 3; ++i) {
		if (!(axes & (1 << i))) continue;
		addPositionControl(imarker, Eigen::Vector3d::Unit(i), orientation_fixed);
	}
}
void addOrientationControl(visualization_msgs::msg::InteractiveMarker &imarker,
									const Eigen::Vector3d &axis, bool orientation_fixed)
{
	visualization_msgs::msg::InteractiveMarkerControl control;
	if (orientation_fixed)
		control.orientation_mode = visualization_msgs::msg::InteractiveMarkerControl::FIXED;
	control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;

	control.orientation << Eigen::Quaterniond::FromTwoVectors(
	                          Eigen::Vector3d::UnitX(), axis);
	imarker.controls.push_back(control);
}
void addOrientationControls(visualization_msgs::msg::InteractiveMarker &imarker,
									 unsigned int axes, bool orientation_fixed = false)
{
	for (unsigned int i=0; i < 3; ++i) {
		if (!(axes & (1 << i))) continue;
		addOrientationControl(imarker, Eigen::Vector3d::Unit(i), orientation_fixed);
	}
}

enum AXES {X = 1, Y = 2, Z = 4, ALL = X | Y | Z};
void make6DofMarker(const geometry_msgs::msg::PoseStamped &stamped, bool ok = true)
{
	visualization_msgs::msg::InteractiveMarker imarker = createInteractiveMarker("6dof marker", stamped);
	double scale = imarker.scale = 0.2;

	visualization_msgs::msg::InteractiveMarkerControl ctrl = createViewPlaneControl(true, true);
	visualization_msgs::msg::Marker m = createSphereMarker(scale * 0.25);
	m.color.r = 0; m.color.g = 1; m.color.b = 1; m.color.a = 0.5;
	ctrl.markers.push_back(m);
	imarker.controls.push_back(ctrl);

	addPositionControls(imarker, AXES::ALL);
	addOrientationControls(imarker, AXES::ALL);

	server->clear();
	server->insert(imarker, &processFeedback);
	server->applyChanges();
}


static
unsigned int computeDepthFromRoot(KDL::SegmentMap::const_iterator el,
                                  const KDL::SegmentMap::const_iterator &root) {
	unsigned int result = 0;
	while (el != root) {
		++result;
		el = el->second.parent;
	}
	return result;
}

const std::string& findEndeffector(const KDL::Tree &tree) {
	const KDL::SegmentMap &segments = tree.getSegments();
	unsigned int maxDepth = 0;
	KDL::SegmentMap::const_iterator eef = segments.end();
	for (KDL::SegmentMap::const_iterator it = segments.begin(), end = segments.end();
	     it != end; ++it) {
		if (it->second.children.size() == 0) {
			unsigned int depth = computeDepthFromRoot(it, tree.getRootSegment());
			if (depth > maxDepth || eef == segments.end()) {
				eef = it;
				maxDepth = depth;
			}
		}
	}
	return eef->first;
}

int main(int argc, char *argv[]) {
	rclcpp::init(argc, argv);
	node = std::make_shared<rclcpp::Node>("ikkdl");

	// fetch KDL tree
	KDL::Tree kdl_tree;
	std::string robot_description;
	node->declare_parameter<std::string>("robot_description", "");
	node->get_parameter("robot_description", robot_description);
	if (!kdl_parser::treeFromString(robot_description, kdl_tree))
	{
		RCLCPP_ERROR(node->get_logger(), "Could not initialize KDL tree");
		return EXIT_FAILURE;
	}

	const std::string &eef = findEndeffector(kdl_tree);
	KDL::Chain kdl_chain;
	if (!kdl_tree.getChain(kdl_tree.getRootSegment()->first, eef, kdl_chain)) {
		RCLCPP_ERROR_STREAM(node->get_logger(), "Could not find chain to " << eef);
		return EXIT_FAILURE;
	}

	// joint state publisher
	auto jsp = node->create_publisher<sensor_msgs::msg::JointState>("joint_states", 1);
	// init joint_state message
	auto js_msg = init_message(kdl_chain);

	// run interactive marker server
	server = std::make_shared<interactive_markers::InteractiveMarkerServer>("ikkdl", node);

	// initialize marker with end-effector pose from forward kinematics
	KDL::ChainFkSolverPos_recursive fk = KDL::ChainFkSolverPos_recursive(kdl_chain);
	KDL::JntArray kdl_joints = KDL::JntArray(kdl_chain.getNrOfJoints());
	KDL::Frame kdl_pose;
	fk.JntToCart(kdl_joints, kdl_pose);

	geometry_msgs::msg::PoseStamped stamped;
	stamped.header.frame_id = kdl_tree.getRootSegment()->first;
	stamped.pose = tf2::toMsg(kdl_pose);

	make6DofMarker(stamped);

	// set initial pose
	marker_feedback.pose = stamped.pose;

	KDL::JntArray joints(kdl_chain.getNrOfJoints());

	// run controller
	rclcpp::Rate rate(50); // 50 hz update rate
	while (rclcpp::ok())
	{
		// TODO: compute twist that moves current end-effector pose into target pose

		// TODO: perform inverse velocity kinematics

		// fill + publish ros joint_state message
		update_message(js_msg, joints);
		jsp->publish(js_msg);

		// process ros messages
		rclcpp::spin_some(node);
		rate.sleep();
	}

	server.reset();
	rclcpp::shutdown();

	return EXIT_SUCCESS;
}
