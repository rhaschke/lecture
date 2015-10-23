#include "QuaternionWidget.h"
#include "ui_quaternion.h"

QuaternionWidget::QuaternionWidget(QWidget *parent) :
   QWidget(parent), _ui(new Ui::QuaternionWidget)
{
	_ui->setupUi(this);
}
