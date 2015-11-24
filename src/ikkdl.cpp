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
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <angles/angles.h>
#include <Eigen/Eigen>
#include <interactive_markers/interactive_marker_server.h>
#include <boost/make_shared.hpp>
#include <tf/tf.h>
#include <tf_conversions/tf_kdl.h>

namespace vm = visualization_msgs;

boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
vm::InteractiveMarkerFeedback marker_feedback;

sensor_msgs::JointState init_message(const KDL::Chain &chain) {
	sensor_msgs::JointState msg;
	for (unsigned int i=0; i < chain.getNrOfSegments(); ++i) {
		KDL::Segment segment = chain.getSegment(i);
		KDL::Joint joint = segment.getJoint();
		if (joint.getType() == KDL::Joint::None) continue;
		msg.name.push_back(joint.getName());
		msg.position.push_back(0);
	}
	return msg;
}

void update_message(sensor_msgs::JointState &msg,
                    const KDL::JntArray &joints) {
	msg.header.stamp = ros::Time::now();
	Eigen::Map<Eigen::VectorXd>(msg.position.data(), msg.position.size()) = joints.data;
}

void processFeedback( const vm::InteractiveMarkerFeedbackConstPtr &feedback )
{
	marker_feedback = *feedback;
}

void operator<< (geometry_msgs::Point &pos, const Eigen::Vector3d &p) {
	pos.x = p[0];
	pos.y = p[1];
	pos.z = p[2];
}
void operator<< (geometry_msgs::Quaternion &quat, const Eigen::Quaterniond &q)
{
	quat.w = q.w();
	quat.x = q.x();
	quat.y = q.y();
	quat.z = q.z();
}

visualization_msgs::InteractiveMarker
createInteractiveMarker(const std::string &name,
                        const geometry_msgs::PoseStamped &stamped)
{
	visualization_msgs::InteractiveMarker imarker;
	imarker.name = name;
	imarker.header = stamped.header;
	imarker.pose = stamped.pose;
	return imarker;
}
visualization_msgs::Marker
createSphereMarker(double radius)
{
	visualization_msgs::Marker marker;

	marker.type = visualization_msgs::Marker::SPHERE;
	marker.scale.x = radius * 2.0;
	marker.scale.y = radius * 2.0;
	marker.scale.z = radius * 2.0;

	return marker;
}
visualization_msgs::InteractiveMarkerControl
createViewPlaneControl(bool position, bool orientation)
{
	visualization_msgs::InteractiveMarkerControl control;
	control.orientation_mode = visualization_msgs::InteractiveMarkerControl::VIEW_FACING;

	if (position && orientation)
		control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D;
   else if (orientation)
		control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_3D;
	else
		control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_3D;

	control.independent_marker_orientation = true;
	control.always_visible = false;
	control.name = "move";

	return control;
}
void addPositionControl(visualization_msgs::InteractiveMarker& imarker,
                        const Eigen::Vector3d &axis, bool orientation_fixed) {
	visualization_msgs::InteractiveMarkerControl control;
	if (orientation_fixed)
		control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;
	control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;

	control.orientation << Eigen::Quaterniond::FromTwoVectors(
	                          Eigen::Vector3d::UnitX(), axis);
	imarker.controls.push_back(control);
}
void addPositionControls(visualization_msgs::InteractiveMarker& imarker,
                         unsigned int axes, bool orientation_fixed = false)
{
	for (unsigned int i=0; i < 3; ++i) {
		if (!(axes & (1 << i))) continue;
		addPositionControl(imarker, Eigen::Vector3d::Unit(i), orientation_fixed);
	}
}
void addOrientationControl(visualization_msgs::InteractiveMarker& imarker,
                           const Eigen::Vector3d &axis, bool orientation_fixed) {
	visualization_msgs::InteractiveMarkerControl control;
	if (orientation_fixed)
		control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;
	control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;

	control.orientation << Eigen::Quaterniond::FromTwoVectors(
	                          Eigen::Vector3d::UnitX(), axis);
	imarker.controls.push_back(control);
}
void addOrientationControls(visualization_msgs::InteractiveMarker& imarker,
                            unsigned int axes, bool orientation_fixed = false)
{
	for (unsigned int i=0; i < 3; ++i) {
		if (!(axes & (1 << i))) continue;
		addOrientationControl(imarker, Eigen::Vector3d::Unit(i), orientation_fixed);
	}
}

enum AXES {X = 1, Y = 2, Z = 4, ALL = X | Y | Z};
void make6DofMarker(const geometry_msgs::PoseStamped &stamped, bool ok = true)
{
	vm::InteractiveMarker imarker = createInteractiveMarker("6dof marker", stamped);
	double scale = imarker.scale = 0.2;

	visualization_msgs::InteractiveMarkerControl ctrl = createViewPlaneControl(true, true);
	visualization_msgs::Marker m = createSphereMarker(scale * 0.25);
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
	std::string tip_frame;
	ros::init (argc, argv, "ikkdl");
	ros::NodeHandle nh;

	// fetch KDL tree
	KDL::Tree kdl_tree;
	if (!kdl_parser::treeFromParam("robot_description", kdl_tree)) {
		ROS_ERROR("Could not initialize KDL tree");
		return EXIT_FAILURE;
	}

	const std::string &eef = findEndeffector(kdl_tree);
	KDL::Chain kdl_chain;
	if (!kdl_tree.getChain(kdl_tree.getRootSegment()->first, eef, kdl_chain)) {
		ROS_ERROR_STREAM("Could not find chain to " << eef);
		return EXIT_FAILURE;
	}

	// joint state publisher
	auto jsp = nh.advertise<sensor_msgs::JointState>("joint_states", 1);
	// init joint_state message
	auto js_msg = init_message(kdl_chain);

	// run interactive marker server
	server.reset(new interactive_markers::InteractiveMarkerServer("ikkdl","",false));

	// initialize marker with end-effector pose from forward kinematics
	KDL::ChainFkSolverPos_recursive fk = KDL::ChainFkSolverPos_recursive(kdl_chain);
	KDL::JntArray kdl_joints = KDL::JntArray(kdl_chain.getNrOfJoints());
	KDL::Frame kdl_pose;
	fk.JntToCart(kdl_joints, kdl_pose);

	tf::Pose tf_pose;
	tf::poseKDLToTF(kdl_pose, tf_pose);

	geometry_msgs::PoseStamped stamped;
	stamped.header.frame_id = kdl_tree.getRootSegment()->first;
	tf::poseTFToMsg(tf_pose, stamped.pose);

	make6DofMarker(stamped);

	// set initial pose
	marker_feedback.pose = stamped.pose;

	KDL::JntArray joints(kdl_chain.getNrOfJoints());

	// run controller
	ros::Rate rate(50); // 50 hz update rate
	while (ros::ok()) {
		// TODO: compute twist that moves current end-effector pose into target pose

		// TODO: perform inverse velocity kinematics

		// fill + publish ros joint_state message
		update_message(js_msg, joints);
		jsp.publish(js_msg);

		// process ros messages
		ros::spinOnce();
		rate.sleep();
	}

	server.reset();

	return EXIT_SUCCESS;
}
