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
	/// quaternion value has changed
	void valueChanged(const Eigen::Quaterniond &q);
	/// euler axis selection changed
	void axesChanged(uint a1, uint a2, uint a3);

public slots:
	void setValue(const Eigen::Quaterniond &q);
	void setEulerAngles(double e1, double e2, double e3);
	void setEulerAxes(uint a1, uint a2, uint a3);

public:
	Eigen::Quaterniond _q;

	QuaternionWidget *_qw;
	EulerWidget      *_ew;
};
