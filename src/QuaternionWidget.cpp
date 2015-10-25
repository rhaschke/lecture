#include "QuaternionWidget.h"
#include "ui_quaternion.h"

QuaternionWidget::QuaternionWidget(QWidget *parent) :
   QWidget(parent), _ui(new Ui::QuaternionWidget)
{
	qRegisterMetaType<Eigen::Quaterniond>("Eigen::Quaterniond");

	_ui->setupUi(this);
}

void QuaternionWidget::setValue(const Eigen::Quaterniond &q)
{
	if (!_q.isApprox(q)) {
		_ui->w->setText(QString::number(q.w()));
		_ui->x->setText(QString::number(q.x()));
		_ui->y->setText(QString::number(q.y()));
		_ui->z->setText(QString::number(q.z()));
		emit valueChanged(q);
	}
	_q = q;
}

const Eigen::Quaterniond &QuaternionWidget::value() const
{
	return _q;
}
