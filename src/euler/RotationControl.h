#pragma once

#include <QGroupBox>
#include <Eigen/Geometry>
#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>

class QuaternionWidget;
class EulerWidget;

/** The RotationControl widget groups a QuaternionWidget and an EulerWidget.
 *  It also creates an interactive marker to allow modifying the orientation in rviz.
 */
class RotationControl : public QGroupBox
{
	Q_OBJECT
public:
	explicit RotationControl(const std::string &_title,
	                         const Eigen::Vector3d &position,
	                         const QColor &color,
	                         boost::shared_ptr<interactive_markers::InteractiveMarkerServer> &_server,
	                         QWidget *parent = 0);

	/// get the current orientation as a quaternion
	const Eigen::Quaterniond& value() const;
	/// get the current orientation as Euler angles
	const Eigen::Vector3d eulerAngles() const;

signals: // signals are emitted when some important event happens
	/// quaternion value has changed
	void valueChanged(const Eigen::Quaterniond &q);
	/// euler axis selection changed (x=0, y=1, z=2)
	void axesChanged(uint a1, uint a2, uint a3);

public slots: // slots can connect to signals and thus get called when these are emitted
	/// set the orientation from the given quaternion
	void setValue(const Eigen::Quaterniond &q, bool update_server=true);
	/// set the orientation from the given Euler angles
	void setEulerAngles(double e1, double e2, double e3);
	/// specify a new set of Euler axes to be used (x=0, y=1, z=2)
	void setEulerAxes(uint a1, uint a2, uint a3);

private:
	void setupUi();
	void createInteractiveMarker(const Eigen::Vector3d &position, const QColor &color);
	void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr&);

private:
	Eigen::Quaterniond _q; ///< current orientation value
	boost::shared_ptr<interactive_markers::InteractiveMarkerServer> _server;
	std::string _title;        ///< title of the group box (and marker)
	geometry_msgs::Pose _pose; ///< pose of the marker, orientation kept in sync with _q

	QuaternionWidget *_qw;
	EulerWidget      *_ew;
};
