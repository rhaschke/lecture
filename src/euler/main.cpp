#include "MainWindow.h"

#include <QApplication>
#include <QTimer>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<rclcpp::Node>("euler");

	QApplication app(argc, argv);
	MainWindow w(node);
	w.show();

	QTimer ros_shutdown_watchdog;
	ros_shutdown_watchdog.setInterval(100);
	QObject::connect(&ros_shutdown_watchdog, &QTimer::timeout, [&app]()
						  { if (!rclcpp::ok()) app.quit(); });
	ros_shutdown_watchdog.start();

	int ret = app.exec();
	rclcpp::shutdown();
	return ret;
}
