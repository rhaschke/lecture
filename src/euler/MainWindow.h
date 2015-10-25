#pragma once

#include <QMainWindow>
#include <memory>
#include <thread>

#include <interactive_markers/interactive_marker_server.hpp>
#include <rclcpp/rclcpp.hpp>

class RotationControl;

class MainWindow : public QMainWindow
{
	Q_OBJECT
public:
	explicit MainWindow(const rclcpp::Node::SharedPtr &node, QWidget *parent = 0);
	~MainWindow();

signals:

public slots:
	void updateFrames();

private:
	void setupUi();

private:
	rclcpp::Node::SharedPtr node;
	std::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
	rclcpp::executors::SingleThreadedExecutor executor;
	std::thread spin_thread;

	RotationControl *frame1;
	RotationControl *frame2;
	RotationControl *frame1p2;
	RotationControl *frame1c2;
};
