#include <QtCore/QCoreApplication>

int main(int argc, char* argv[])
{
	{
		QCoreApplication app(argc, argv);
		QString app_name("test coreapplication");
		app.setApplicationName(app_name);
	}

	return 0;
}