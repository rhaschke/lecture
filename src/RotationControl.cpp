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
}

void RotationControl::setValue(const Eigen::Quaterniond &q)
{
	if (!_q.isApprox(q)) {
		_q = q;
		_qw->setValue(q);
		_ew->setValue(q);
		emit valueChanged(q);
	}
}
