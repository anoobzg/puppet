#include "operation.h"

Operation::Operation()
{

}

Operation::~Operation()
{

}

bool Operation::ReleaseOnFrame(int frame)
{
	return false;
}

bool Operation::OnMouse(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	return false;
}

const char* Operation::OperationName()
{
	return "Operation";
}