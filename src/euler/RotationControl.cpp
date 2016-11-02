#include "RotationControl.h"
#include "QuaternionWidget.h"
#include "EulerWidget.h"

#include <QBoxLayout>
#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>

RotationControl::RotationControl(const std::string &title,
                                 const Eigen::Vector3d &position, const QColor &color,
                                 boost::shared_ptr<interactive_markers::InteractiveMarkerServer> &server,
                                 QWidget *parent) :
   QGroupBox(QString::fromStdString(title), parent), _server(server), _title(title)
{
	qRegisterMetaType<Eigen::Quaterniond>("Eigen::Quaterniond");

	setupUi();
	createInteractiveMarker(position, color);
}

void RotationControl::setupUi() {
	_qw = new QuaternionWidget(this);
	_ew = new EulerWidget(this);

	QBoxLayout *layout = new QBoxLayout(QBoxLayout::TopToBottom, this);
	layout->addWidget(_qw);
	layout->addWidget(_ew);
	this->setLayout(layout);

	setValue(Eigen::Quaterniond::Identity());

	connect(_qw, SIGNAL(valueChanged(Eigen::Quaterniond)),
	        this, SLOT(setValue(Eigen::Quaterniond)));
	connect(_ew, SIGNAL(valueChanged(Eigen::Quaterniond)),
	        this, SLOT(setValue(Eigen::Quaterniond)));
	connect(_ew, SIGNAL(axesChanged(uint,uint,uint)),
	        this, SIGNAL(axesChanged(uint,uint,uint)));
}


const Eigen::Quaterniond &RotationControl::value() const {
	return _q;
}

void RotationControl::setValue(const Eigen::Quaterniond &q, bool update_server) {
	if (q.isApprox(_q)) return;
	_q = q;

	_qw->blockSignals(true);
	_ew->blockSignals(true);
	_qw->setValue(q);
	_ew->setValue(q);
	_qw->blockSignals(false);
	_ew->blockSignals(false);

	if (_server && update_server) {
		updatePose(q);
		_server->setPose(_title, _pose);
		_server->applyChanges();
	}

	emit valueChanged(q);
}


void RotationControl::setEulerAxes(uint a1, uint a2, uint a3) {
	_ew->setEulerAxes(a1, a2, a3);
}

const Eigen::Vector3d RotationControl::eulerAngles() const {
	Eigen::Vector3d result;
	_ew->getGuiAngles(result.data());
	return result;
}

void RotationControl::setEulerAngles(double e1, double e2, double e3) {
	_ew->setEulerAngles(e1, e2, e3, true);
}

void RotationControl::updatePose(const Eigen::Quaterniond &q) {
	_pose.orientation.w = q.w();
	_pose.orientation.x = q.x();
	_pose.orientation.y = q.y();
	_pose.orientation.z = q.z();
}

static visualization_msgs::InteractiveMarkerControl createViewPlaneControl() {
	visualization_msgs::InteractiveMarkerControl control;
	control.orientation_mode = visualization_msgs::InteractiveMarkerControl::VIEW_FACING;
	control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_3D;
	control.independent_marker_orientation = true;
	control.always_visible = true;
	control.name = "rotate";

	return control;
}

static visualization_msgs::Marker createBoxMarker(double x, double y, double z,
                                                  const QColor &color) {
	visualization_msgs::Marker marker;

	marker.type = visualization_msgs::Marker::CUBE;
	marker.scale.x = x;
	marker.scale.y = y;
	marker.scale.z = z;

	marker.color.r = color.redF();
	marker.color.g = color.greenF();
	marker.color.b = color.blueF();
	marker.color.a = color.alphaF();

	return marker;
}

void RotationControl::createInteractiveMarker(const Eigen::Vector3d &pos,
                                              const QColor &color) {
	if (!_server) return;

	_pose.position.x = pos[0];
	_pose.position.y = pos[1];
	_pose.position.z = pos[2];
	updatePose(_q);

	visualization_msgs::InteractiveMarker imarker;
	imarker.header.frame_id = "world";
	imarker.header.stamp = ros::Time::now();
	imarker.pose = _pose;
	imarker.name = _title;
	float s = imarker.scale = 0.2;

	visualization_msgs::InteractiveMarkerControl ctrl = createViewPlaneControl();
	ctrl.markers.push_back(createBoxMarker(3*s, 2*s, 1*s, color));
	imarker.controls.push_back(ctrl);

	_server->insert(imarker, boost::bind(&RotationControl::processFeedback, this, _1));
}

void RotationControl::processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
	const geometry_msgs::Quaternion &q = feedback->pose.orientation;
	setValue(Eigen::Quaterniond(q.w, q.x, q.y, q.z), false);
}
