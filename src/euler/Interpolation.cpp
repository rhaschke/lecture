#include "Interpolation.h"
#include <QMutexLocker>

Interpolation::Interpolation(const Eigen::Quaterniond &q1,
                             const Eigen::Quaterniond &q2,
                             QObject* parent)
   :_q1(q1), _q2(q2)
{
	_t = 0.0;
	connect(this, SIGNAL(timeout()), this, SLOT(step()));
}

void Interpolation::setStart(const Eigen::Quaterniond &q)
{
	QMutexLocker lock(&_mutex);
	_q1 = q;
	_t = 0;
}

void Interpolation::setEnd(const Eigen::Quaterniond &q)
{
	QMutexLocker lock(&_mutex);
	_q2 = q;
	_t = 0;
}

void Interpolation::step()
{
	QMutexLocker lock(&_mutex);
	_t += 0.05;
	if (_t > 1.) _t = -1.;
	emit valueChanged(_q1.slerp(std::fabs(_t), _q2));
}
