#include "EulerWidget.h"
#include "ui_euler.h"

#include <angles/angles.h>
#include <QStandardItemModel>
#include <iostream>

// ensure different axes for consecutive operations
static void disableAxis(QComboBox *w, unsigned int axis) {
	const QStandardItemModel* model = qobject_cast<const QStandardItemModel*>(w->model());
	for (unsigned int i=0; i < 3; ++i) {
		QStandardItem* item = model->item(i);
		if (i == axis) {
			item->setFlags(item->flags() & ~Qt::ItemIsEnabled);
			if (w->currentIndex() == axis) w->setCurrentIndex((axis+1) % 3);
		} else {
			item->setFlags(item->flags() | Qt::ItemIsEnabled);
		}
	}
}

EulerWidget::EulerWidget(QWidget *parent) :
   QWidget(parent), _ui(new Ui::EulerWidget)
{
	qRegisterMetaType<Eigen::Quaterniond>("Eigen::Quaterniond");

	_ui->setupUi(this);
	_ui->a1->setCurrentIndex(0);
	_ui->a2->setCurrentIndex(1); disableAxis(_ui->a2, 0);
	_ui->a3->setCurrentIndex(2); disableAxis(_ui->a3, 1);

	// react to axis changes
	connect(_ui->a1, SIGNAL(currentIndexChanged(int)),
	        this, SLOT(axisChanged(int)));
	connect(_ui->a2, SIGNAL(currentIndexChanged(int)),
	        this, SLOT(axisChanged(int)));
	connect(_ui->a3, SIGNAL(currentIndexChanged(int)),
	        this, SLOT(axisChanged(int)));

	// react to angle changes
	connect(_ui->e1, SIGNAL(valueChanged(double)),
	        this, SLOT(angleChanged(double)));
	connect(_ui->e2, SIGNAL(valueChanged(double)),
	        this, SLOT(angleChanged(double)));
	connect(_ui->e3, SIGNAL(valueChanged(double)),
	        this, SLOT(angleChanged(double)));
}

void EulerWidget::getGuiAxes(int a[3]) const {
	a[0] = _ui->a1->currentIndex();
	a[1] = _ui->a2->currentIndex();
	a[2] = _ui->a3->currentIndex();
}

void EulerWidget::getGuiAngles(double e[3]) const {
	e[0] = angles::from_degrees(_ui->e1->value());
	e[1] = angles::from_degrees(_ui->e2->value());
	e[2] = angles::from_degrees(_ui->e3->value());
}

template <typename VectorType>
void EulerWidget::setGuiAngles(const VectorType &e)
{
	_ui->e1->setValue(angles::to_degrees(e[0]));
	_ui->e2->setValue(angles::to_degrees(e[1]));
	_ui->e3->setValue(angles::to_degrees(e[2]));
}


void EulerWidget::axisChanged(int axis) {
	Eigen::Quaterniond old = _q;

	// ensure different axes for consecutive operations
	QComboBox* origin = dynamic_cast<QComboBox*>(sender());
	if (origin == _ui->a1) disableAxis(_ui->a2, axis);
	if (origin == _ui->a2) disableAxis(_ui->a3, axis);

	setValue(old);
}

void EulerWidget::angleChanged(double angle) {
	EulerWidget::setValue(value());
}


void EulerWidget::setValue(const Eigen::Quaterniond &q)
{
	// ensure different axes for consecutive operations
	int a[3]; getGuiAxes(a);
	this->blockSignals(true);
	setGuiAngles(q.matrix().eulerAngles(a[0],a[1],a[2]));
	this->blockSignals(false);

	if (!_q.isApprox(q)) {
		emit valueChanged(q);
	}
	_q = q;
}

const Eigen::Quaterniond EulerWidget::value() const
{
	int a[3]; getGuiAxes(a);
	double e[3]; getGuiAngles(e);
	return Eigen::AngleAxisd(e[0], Eigen::Vector3d::Unit(a[0]))
	      *Eigen::AngleAxisd(e[1], Eigen::Vector3d::Unit(a[1]))
	      *Eigen::AngleAxisd(e[2], Eigen::Vector3d::Unit(a[2]));
}

