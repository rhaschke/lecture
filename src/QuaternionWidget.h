#pragma once

#include <QWidget>
#include <Eigen/Geometry>

namespace Ui {
class QuaternionWidget;
}

class QuaternionWidget : public QWidget
{
	Q_OBJECT
public:
	explicit QuaternionWidget(QWidget *parent = 0);

	const Eigen::Quaterniond& value() const;

signals:
	/// quaternion value has changed
	void valueChanged(const Eigen::Quaterniond &q);

public slots:
	void setValue(const Eigen::Quaterniond &q);

private:
	Eigen::Quaterniond _q;
	Ui::QuaternionWidget *_ui;
};
