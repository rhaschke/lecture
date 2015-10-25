#include "EulerWidget.h"
#include "ui_euler.h"

EulerWidget::EulerWidget(QWidget *parent) :
   QWidget(parent), _ui(new Ui::EulerWidget)
{
	qRegisterMetaType<Eigen::Quaterniond>("Eigen::Quaterniond");

	_ui->setupUi(this);
}
