#include "deletealloperation.h"
#include <osgWrapper/ProgramManager.h>

DeleteAllOperation::DeleteAllOperation(int frame, FeedGeometry* geometry)
	:m_geometry(geometry), m_frame(frame)
{
	SetRenderProgram("feed_deleteall");

	AddChild(m_geometry);
}

DeleteAllOperation::~DeleteAllOperation()
{

}

const char* DeleteAllOperation::OperationName()
{
	return "DeleteAllOperation";
}

bool DeleteAllOperation::ReleaseOnFrame(int frame)
{
	return frame - m_frame >= 3;
}