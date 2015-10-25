#pragma once

#include <QWidget>
#include <Eigen/Geometry>

class QuaternionWidget;
class EulerWidget;

class RotationControl : public QWidget
{
	Q_OBJECT
public:
	explicit RotationControl(QWidget *parent = 0,
	                         const QString &title="");

	const Eigen::Quaterniond& value() const;
	const Eigen::Vector3d eulerAngles() const;

signals:
	void valueChanged(const Eigen::Quaterniond &q);

public slots:
	void setValue(const Eigen::Quaterniond &q);
	void setEulerAngles(double e1, double e2, double e3);

public:
	Eigen::Quaterniond _q;

	QuaternionWidget *_qw;
	EulerWidget      *_ew;
};
