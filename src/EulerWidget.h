#pragma once

#include <QWidget>
#include <Eigen/Geometry>

namespace Ui {
class EulerWidget;
}

class EulerWidget : public QWidget
{
	Q_OBJECT
public:
	explicit EulerWidget(QWidget *parent = 0);

	const Eigen::Quaterniond &value() const;

	/// retrieve indices of axes selected in GUI
	void getGuiAxes(int a[]) const;
	/// retrieve angles from GUI
	void getGuiAngles(double e[]) const;

signals:
	/// quaternion value has changed
	void valueChanged(const Eigen::Quaterniond &q);

public slots:
	void setValue(const Eigen::Quaterniond &q);
	void setEulerAngles(double e1, double e2, double e3);

protected slots:
	void axisChanged(int axis);
	void angleChanged(double angle);

private slots:
	void updateAngles();

private:
	Eigen::Quaterniond _q;
	Ui::EulerWidget *_ui;
};
