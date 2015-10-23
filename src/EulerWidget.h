#pragma once

#include <QWidget>

namespace Ui {
class EulerWidget;
}

class EulerWidget : public QWidget
{
	Q_OBJECT
public:
	explicit EulerWidget(QWidget *parent = 0);

signals:

public slots:

private:
	Ui::EulerWidget *_ui;
};
