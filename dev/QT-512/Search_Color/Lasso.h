#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_Lasso.h"
#include "Searcher.h"

class Lasso : public QMainWindow, public SearcherListener
{
	Q_OBJECT

public:
	Lasso(QWidget *parent = Q_NULLPTR);
	virtual ~Lasso();

protected slots:
	void Load();
	void Search();

private:
	void SearchStart();
	void SearchStep(const QImage& image);
	void SearchEnd();
private:
	Ui::LassoClass ui;

	Searcher m_searcher;
	QImage* m_image;
};
