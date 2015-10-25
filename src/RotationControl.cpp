#include "RotationControl.h"
#include "QuaternionWidget.h"
#include "EulerWidget.h"

#include <QBoxLayout>

RotationControl::RotationControl(QWidget *parent, const QString &title) :
   QWidget(parent)
{
	qRegisterMetaType<Eigen::Quaterniond>("Eigen::Quaterniond");

	_qw = new QuaternionWidget(parent);
	_ew = new EulerWidget(parent);

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

void RotationControl::setValue(const Eigen::Quaterniond &q) {
	if (q.isApprox(_q)) return;
	_q = q;

	if (!q.isApprox(_qw->value())) _qw->setValue(q);
	if (!q.isApprox(_ew->value())) _ew->setValue(q);
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
	_ew->setEulerAngles(e1, e2, e3);
}
