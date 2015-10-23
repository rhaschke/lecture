#include "RotationControl.h"
#include "QuaternionWidget.h"
#include "EulerWidget.h"

#include <QBoxLayout>

RotationControl::RotationControl(QWidget *parent, const QString &title) :
   QWidget(parent)
{
	_qw = new QuaternionWidget(parent);
	_ew = new EulerWidget(parent);

	QBoxLayout *layout = new QBoxLayout(QBoxLayout::TopToBottom, this);
	layout->addWidget(_qw);
	layout->addWidget(_ew);
	this->setLayout(layout);
}
