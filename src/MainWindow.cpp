#include "MainWindow.h"
#include "RotationControl.h"

#include <QVBoxLayout>

MainWindow::MainWindow(QWidget *parent) :
   QMainWindow(parent)
{
	QWidget *central = new QWidget(this);

	frame1 = new RotationControl(this);
	frame2 = new RotationControl(this);
	frame1p2 = new RotationControl(this);
	frame1c2 = new RotationControl(this);

	QVBoxLayout *layout = new QVBoxLayout(central);
	layout->addWidget(frame1);
	layout->addWidget(frame2);
	layout->addWidget(frame1p2);
	layout->addWidget(frame1c2);

	this->setCentralWidget(central);
}
