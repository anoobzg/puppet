#include "selectalloperation.h"
#include <osgWrapper/ProgramManager.h>

SelectAllOperation::SelectAllOperation(int frame, FeedGeometry* geometry)
	:m_geometry(geometry), m_frame(frame)
{
	SetRenderProgram("feed_selectall");

	AddChild(m_geometry);
}

SelectAllOperation::~SelectAllOperation()
{

}

const char* SelectAllOperation::OperationName()
{
	return "SelectAllOperation";
}

bool SelectAllOperation::ReleaseOnFrame(int frame)
{
	return frame - m_frame >= 3;
}