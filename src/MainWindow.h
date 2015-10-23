#pragma once

#include <QMainWindow>
class RotationControl;

class MainWindow : public QMainWindow
{
	Q_OBJECT
public:
	explicit MainWindow(QWidget *parent = 0);

signals:

public slots:

public:
	RotationControl *frame1;
	RotationControl *frame2;
	RotationControl *frame1p2;
	RotationControl *frame1c2;
};
