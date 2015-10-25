#include "MainWindow.h"
#include "RotationControl.h"

#include <QVBoxLayout>
#include <Eigen/Core>
#include <ros/ros.h>

MainWindow::MainWindow(QWidget *parent) :
   QMainWindow(parent), spinner(1)
{
	server.reset(new interactive_markers::InteractiveMarkerServer("euler","",false));
	setupUi();

	server->applyChanges();
	spinner.start();
}

MainWindow::~MainWindow()
{
	spinner.stop();
}

void MainWindow::setupUi() {
	QWidget *central = new QWidget(this);

	double s = 0.5;
	QColor grey("grey"), red("red"), green("green");
	frame1 = new RotationControl("frame 1", Eigen::Vector3d(-s,s,0), grey, server, this);
	frame2 = new RotationControl("frame 2", Eigen::Vector3d( s,s,0), grey, server, this);
	frame1p2 = new RotationControl("frame 1+2", Eigen::Vector3d(-s,-s,0), red, server, this);
	frame1c2 = new RotationControl("frame 1*2", Eigen::Vector3d( s,-s,0), green, server, this);

	QVBoxLayout *layout = new QVBoxLayout(central);
	layout->addWidget(frame1);
	layout->addWidget(frame2);
	layout->addWidget(frame1p2);
	layout->addWidget(frame1c2);

	this->setCentralWidget(central);
}
