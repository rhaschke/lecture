#pragma once

#include <QWidget>

class QuaternionWidget;
class EulerWidget;

class RotationControl : public QWidget
{
	Q_OBJECT
public:
	explicit RotationControl(QWidget *parent = 0,
	                         const QString &title="");

signals:

public slots:

public:
	QuaternionWidget *_qw;
	EulerWidget      *_ew;
};
