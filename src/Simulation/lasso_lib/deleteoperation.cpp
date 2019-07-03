#include "deleteoperation.h"
#include <osgWrapper/ProgramManager.h>

DeleteOperation::DeleteOperation(int frame, FeedGeometry* geometry)
	:m_geometry(geometry), m_frame(frame)
{
	SetRenderProgram("feed_delete");

	AddChild(m_geometry);
}

DeleteOperation::~DeleteOperation()
{

}

const char* DeleteOperation::OperationName()
{
	return "DeleteOperation";
}

bool DeleteOperation::ReleaseOnFrame(int frame)
{
	return frame - m_frame >= 3;
}