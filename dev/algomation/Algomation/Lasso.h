#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_Lasso.h"

class Lasso : public QMainWindow
{
	Q_OBJECT

public:
	Lasso(QWidget *parent = Q_NULLPTR);

private:
	Ui::LassoClass ui;
};
