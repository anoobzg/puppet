#include "listener.h"
#include <iostream>

Listener::Listener()
{

}

Listener::~Listener()
{

}

void Listener::ApplicationNameChanged()
{
	std::cout << "application name changed." << std::endl;
}

void Listener::ObjectDestroyed(QObject* object)
{
	std::cout << "object destroyed." << std::endl;
}

#include "moc_listener.cpp"