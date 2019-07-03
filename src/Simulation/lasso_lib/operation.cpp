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

bool Operation::NeedCopy()
{
	return true;
}

const char* Operation::OperationName()
{
	return "Operation";
}

MouseOperation::MouseOperation()
	:m_release(false)
{

}

MouseOperation::~MouseOperation()
{

}

bool MouseOperation::ReleaseOnFrame(int frame)
{
	return m_release;
}

void MouseOperation::SetRelease()
{
	m_release = true;
}

const char* MouseOperation::OperationName()
{
	return "MouseOperation";
}