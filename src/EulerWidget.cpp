#include "EulerWidget.h"
#include "ui_euler.h"

EulerWidget::EulerWidget(QWidget *parent) :
   QWidget(parent), _ui(new Ui::EulerWidget)
{
	_ui->setupUi(this);
}
