#include "QuaternionWidget.h"
#include "ui_quaternion.h"

QuaternionWidget::QuaternionWidget(QWidget *parent) :
   QWidget(parent), _ui(new Ui::QuaternionWidget)
{
	qRegisterMetaType<Eigen::Quaterniond>("Eigen::Quaterniond");

	_ui->setupUi(this);
}
