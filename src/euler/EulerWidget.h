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
	enum Axis {X = 0, Y = 1, Z = 2};

	explicit EulerWidget(QWidget *parent = 0);

	const Eigen::Quaterniond &value() const;

	/// retrieve indices of axes selected in GUI
	void getGuiAxes(uint a[3]) const;
	/// retrieve angles from GUI
	void getGuiAngles(double e[]) const;

signals:
	/// quaternion value has changed
	void valueChanged(const Eigen::Quaterniond &q);
	/// euler axis selection changed
	void axesChanged(uint a1, uint a2, uint a3);

public slots:
	void setValue(const Eigen::Quaterniond &q);
	void setEulerAngles(double e1, double e2, double e3, bool normalize);
	void setEulerAxes(uint a1, uint a2, uint a3);

protected slots:
	void axisChanged(int axis);
	void angleChanged();

private slots:
	void updateAngles();

private:
	Eigen::Quaterniond _q;
	Ui::EulerWidget *_ui;
};
