#pragma once

#include <QWidget>

namespace Ui {
class QuaternionWidget;
}

class QuaternionWidget : public QWidget
{
	Q_OBJECT
public:
	explicit QuaternionWidget(QWidget *parent = 0);

signals:

public slots:

private:
	Ui::QuaternionWidget *_ui;

};
