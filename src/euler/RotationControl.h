#pragma once

#include <QGroupBox>
#include <Eigen/Geometry>
#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>

class QuaternionWidget;
class EulerWidget;

class RotationControl : public QGroupBox
{
	Q_OBJECT
public:
	explicit RotationControl(const std::string &_title,
	                         const Eigen::Vector3d &position,
	                         const QColor &color,
	                         boost::shared_ptr<interactive_markers::InteractiveMarkerServer> &_server,
	                         QWidget *parent = 0);

	const Eigen::Quaterniond& value() const;
	const Eigen::Vector3d eulerAngles() const;

signals:
	/// quaternion value has changed
	void valueChanged(const Eigen::Quaterniond &q);
	/// euler axis selection changed
	void axesChanged(uint a1, uint a2, uint a3);

public slots:
	void setValue(const Eigen::Quaterniond &q, bool update_server=true);
	void setEulerAngles(double e1, double e2, double e3);
	void setEulerAxes(uint a1, uint a2, uint a3);

private:
	void setupUi();
	void createInteractiveMarker(const Eigen::Vector3d &position, const QColor &color);
	void updatePose(const Eigen::Quaterniond &q);
	void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr&);

private:
	Eigen::Quaterniond  _q;
	boost::shared_ptr<interactive_markers::InteractiveMarkerServer> _server;
	std::string         _title;
	geometry_msgs::Pose _pose;

	QuaternionWidget *_qw;
	EulerWidget      *_ew;
};
