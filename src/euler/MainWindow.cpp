// include headers declaring the used class interfaces
#include "MainWindow.h"
#include "RotationControl.h"

// these are system includes (from Qt, Eigen, ROS)
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

// keep axes in sync between the two RotationControl widgets f1 and f2
static void linkAxes(RotationControl *f1, RotationControl *f2) {
	// f1 -> f2
	QObject::connect(f1, SIGNAL(axesChanged(uint,uint,uint)),
	                 f2, SLOT(setEulerAxes(uint,uint,uint)));
	// f2 -> f1
	QObject::connect(f2, SIGNAL(axesChanged(uint,uint,uint)),
	                 f1, SLOT(setEulerAxes(uint,uint,uint)));
}

void MainWindow::setupUi() {
	QWidget *central = new QWidget(this);

	// create 4 RotationControl widgets
	double s = 0.5;
	QColor grey("grey"), red("red"), green("green");
	frame1 = new RotationControl("frame 1", Eigen::Vector3d(-s,s,0), grey, server, this);
	frame2 = new RotationControl("frame 2", Eigen::Vector3d( s,s,0), grey, server, this);
	frame1p2 = new RotationControl("frame 1+2", Eigen::Vector3d(-s,-s,0), red, server, this);
	frame1c2 = new RotationControl("frame 1*2", Eigen::Vector3d( s,-s,0), green, server, this);

	// add those widgets to the vertical layout of the MainWindow's central widget
	QVBoxLayout *layout = new QVBoxLayout(central);
	layout->addWidget(frame1);
	layout->addWidget(frame2);
	layout->addWidget(frame1p2);
	layout->addWidget(frame1c2);

	this->setCentralWidget(central);

	// keep Euler axes in sync between all widgets
	linkAxes(frame1, frame2);
	linkAxes(frame1, frame1p2);
	linkAxes(frame1, frame1c2);
}
