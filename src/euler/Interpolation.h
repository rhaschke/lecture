#pragma once

#include <QTimer>
#include <QMutex>
#include <Eigen/Geometry>

class Interpolation : public QTimer
{
	Q_OBJECT
public:
	Interpolation(const Eigen::Quaterniond &q1 = Eigen::Quaterniond::Identity(),
	              const Eigen::Quaterniond &q2 = Eigen::Quaterniond::Identity(),
	              QObject* parent = Q_NULLPTR);

public slots:
	void setStart(const Eigen::Quaterniond &q1);
	void setEnd(const Eigen::Quaterniond &q2);

signals:
	void valueChanged(const Eigen::Quaterniond &q);

private slots:
	void step();

private:
	QMutex _mutex;
	Eigen::Quaterniond _q1, _q2;
	double _t;
};
