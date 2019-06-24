#pragma once
#include <QtCore/QObject>

class Listener : public QObject
{
	Q_OBJECT
public:
	Listener();
	~Listener();

public slots:
	void ApplicationNameChanged();
	void ObjectDestroyed(QObject* object);
};