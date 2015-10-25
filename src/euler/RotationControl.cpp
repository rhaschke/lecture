#include "RotationControl.h"
#include "QuaternionWidget.h"
#include "EulerWidget.h"

#include <QBoxLayout>
#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>

// update pose's orientation from the given quaternion q
// pose is passed by reference (&), so the changes are visible outside this function
static void updatePose(geometry_msgs::Pose &pose,
							  const Eigen::Quaterniond &q)
{
	pose.orientation.w = q.w();
	pose.orientation.x = q.x();
	pose.orientation.y = q.y();
	pose.orientation.z = q.z();
}

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

	// Keep orientations stored in EulerWidget/QuaternionWidget in sync with our own quaternion value
	// To this end, connect those widgets' valueChanged signals to our setValue slot
	connect(_qw, SIGNAL(valueChanged(Eigen::Quaterniond)),
	        this, SLOT(setValue(Eigen::Quaterniond)));
	connect(_ew, SIGNAL(valueChanged(Eigen::Quaterniond)),
	        this, SLOT(setValue(Eigen::Quaterniond)));

	// Forward the axesChanged signal from the EulerWidget
	connect(_ew, SIGNAL(axesChanged(uint,uint,uint)),
	        this, SIGNAL(axesChanged(uint,uint,uint)));
}


const Eigen::Quaterniond &RotationControl::value() const {
	return _q;
}

void RotationControl::setValue(const Eigen::Quaterniond &q, bool update_server) {
	if (q.isApprox(_q))
		return; // avoid infinite recursion
	_q = q;

	// update children widgets' orientation values, ignoring their valueChanged signals
	_qw->blockSignals(true);
	_ew->blockSignals(true);
	_qw->setValue(q);
	_ew->setValue(q);
	_qw->blockSignals(false);
	_ew->blockSignals(false);

	if (_server && update_server) {
		updatePose(_pose, q);
		_server->setPose(_title, _pose);
		_server->applyChanges();
	}

	// emit valueChanged signal to inform listeners about the new value
	emit valueChanged(q);
}


void RotationControl::setEulerAxes(uint a1, uint a2, uint a3) {
	_ew->setEulerAxes(a1, a2, a3); // forward to EulerWidget
}

const Eigen::Vector3d RotationControl::eulerAngles() const {
	Eigen::Vector3d result;
	_ew->getGuiAngles(result.data());
	return result;
}

void RotationControl::setEulerAngles(double e1, double e2, double e3) {
	_ew->setEulerAngles(e1, e2, e3, true); // forward to EulerWidget
}

/// create an interactive marker control allowing rotation only (with left mouse button)
static visualization_msgs::InteractiveMarkerControl createViewPlaneControl() {
	visualization_msgs::InteractiveMarkerControl control;
	control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_3D;
	control.always_visible = true;
	control.name = "rotate"; // identify the control (for use in feedback callback)

	return control;
}

/// create a visual marker, here a box of given dimensions and color
static visualization_msgs::Marker createBoxMarker(double x, double y, double z,
                                                  const QColor &color) {
	visualization_msgs::Marker marker;

	marker.type = visualization_msgs::Marker::CUBE; // create a box
	marker.scale.x = x;										// pass box dimensions via scale
	marker.scale.y = y;
	marker.scale.z = z;

	marker.color.r = color.redF(); // pass color components in range 0..1
	marker.color.g = color.greenF();
	marker.color.b = color.blueF();
	marker.color.a = color.alphaF();

	return marker;
}

/// create a visual marker, here an arrow of given size (scale), direction, and color
static visualization_msgs::Marker createArrowMarker(double scale,
                                                    const Eigen::Vector3d &dir,
                                                    const QColor &color) {
	visualization_msgs::Marker marker;

	marker.type = visualization_msgs::Marker::ARROW;
	marker.scale.x = scale;
	marker.scale.y = 0.1*scale;
	marker.scale.z = 0.1*scale;

	// set arrow's orientation from given direction
	updatePose(marker.pose,
	           Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitX(), dir));

	marker.color.r = color.redF();
	marker.color.g = color.greenF();
	marker.color.b = color.blueF();
	marker.color.a = color.alphaF();

	return marker;
}

/// create an interactive marker located at given position
void RotationControl::createInteractiveMarker(const Eigen::Vector3d &pos,
                                              const QColor &color) {
	if (!_server)
		return; // no server, no marker

	// initialize _pose from position and current quaternion
	_pose.position.x = pos[0];
	_pose.position.y = pos[1];
	_pose.position.z = pos[2];
	updatePose(_pose, _q);

	// configure the interactive marker's pose, name, and size (scale)
	visualization_msgs::InteractiveMarker imarker;
	imarker.header.frame_id = "world";
	imarker.header.stamp = ros::Time::now();
	imarker.pose = _pose;
	imarker.name = _title;
	float s = imarker.scale = 0.2;

	// an interactive marker consists of one or more controls determining
	// the type of interaction (e.g. rotation, translation, etc.)
	visualization_msgs::InteractiveMarkerControl ctrl = createViewPlaneControl();
	// each control can have multiple markers determining its visual appearance
	ctrl.markers.push_back(createBoxMarker(3*s, 2*s, 1*s, color));
	// add an arrow for each axis
	ctrl.markers.push_back(createArrowMarker(3*s, Eigen::Vector3d::UnitX(), QColor("red")));
	ctrl.markers.push_back(createArrowMarker(3*s, Eigen::Vector3d::UnitY(), QColor("green")));
	ctrl.markers.push_back(createArrowMarker(3*s, Eigen::Vector3d::UnitZ(), QColor("blue")));

	// add the control to the interactive marker
	imarker.controls.push_back(ctrl);
	// add the interactive marker to the server, registering processFeedback as the update callback
	_server->insert(imarker, boost::bind(&RotationControl::processFeedback, this, _1));
}

/// callback for interactive marker updates, providing the new pose and other interaction info
void RotationControl::processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
	const geometry_msgs::Quaternion &q = feedback->pose.orientation;
	setValue(Eigen::Quaterniond(q.w, q.x, q.y, q.z), false);
}
