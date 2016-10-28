#include "QuaternionWidget.h"
#include "ui_quaternion.h"

QuaternionWidget::QuaternionWidget(QWidget *parent) :
   QWidget(parent), _ui(new Ui::QuaternionWidget) {
	qRegisterMetaType<Eigen::Quaterniond>("Eigen::Quaterniond");

	_ui->setupUi(this);
	_q = Eigen::Quaterniond::Identity();
	updateDisplay();
}

void QuaternionWidget::setValue(const Eigen::Quaterniond &q) {
	if (q.isApprox(_q, 1e-5)) return;
	_q = q;
	updateDisplay();
	emit valueChanged(q);
}

const Eigen::Quaterniond &QuaternionWidget::value() const {
	return _q;
}


void QuaternionWidget::updateDisplay() {
	_ui->w->setText(QString::number(_q.w()));
	_ui->x->setText(QString::number(_q.x()));
	_ui->y->setText(QString::number(_q.y()));
	_ui->z->setText(QString::number(_q.z()));
}
