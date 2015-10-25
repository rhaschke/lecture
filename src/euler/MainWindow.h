#pragma once

#include <QMainWindow>
#include <interactive_markers/interactive_marker_server.h>

class RotationControl;

class MainWindow : public QMainWindow
{
	Q_OBJECT
public:
	explicit MainWindow(QWidget *parent = 0);
	~MainWindow();

signals:

public slots:
	void updateFrames();

private:
	void setupUi();

private:
	boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
	ros::AsyncSpinner spinner;

	RotationControl *frame1;
	RotationControl *frame2;
	RotationControl *frame1p2;
	RotationControl *frame1c2;
};
