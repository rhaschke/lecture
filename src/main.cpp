#include <QApplication>
#include "MainWindow.h"

#include <ros/ros.h>

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "euler");

	QApplication app(argc, argv);
	MainWindow w;

	w.show();
	return app.exec();
}
