#include "Lasso.h"
#include <QFileDialog>

Lasso::Lasso(QWidget *parent)
	: QMainWindow(parent), m_image(NULL)
{
	ui.setupUi(this);

	connect(ui.actionINPUT, SIGNAL(triggered()), this, SLOT(Load()));
	connect(ui.actionSEARCH, SIGNAL(triggered()), this, SLOT(Search()));

	m_searcher.SetListener(this);
}

Lasso::~Lasso()
{

}

void Lasso::Load()
{
	QString file_name = QFileDialog::getOpenFileName(this);
	if (file_name.isEmpty()) return;

	if (m_image)
	{
		m_searcher.SetImage(NULL);
		delete m_image;
		m_image = NULL;
	}

	m_image = new QImage(file_name);
	ui.label->setPixmap(QPixmap::fromImage(*m_image));

	if ((m_image->width() > 0) && (m_image->height() > 0))
		m_searcher.SetImage(m_image);
}

void Lasso::Search()
{
	m_searcher.Start();
}

void Lasso::SearchStart()
{
	ui.actionINPUT->setEnabled(false);
	ui.actionSEARCH->setEnabled(false);
}

void Lasso::SearchStep(const QImage& image)
{
	ui.label_2->setPixmap(QPixmap::fromImage(image));
}

void Lasso::SearchEnd()
{
	ui.actionINPUT->setEnabled(true);
	ui.actionSEARCH->setEnabled(true);
}