#include "Lasso.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	Lasso w;
	w.show();
	return a.exec();
}
