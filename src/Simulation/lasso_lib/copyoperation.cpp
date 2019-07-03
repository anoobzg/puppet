#include "copyoperation.h"
#include <osgWrapper/ProgramManager.h>

CopyOperation::CopyOperation(int frame, FeedGeometry* geometry)
	:m_geometry(geometry), m_frame(frame)
{
	SetRenderProgram("feed_copy");

	AddChild(m_geometry);
}

CopyOperation::~CopyOperation()
{

}

const char* CopyOperation::OperationName()
{
	return "CopyOperation";
}

bool CopyOperation::ReleaseOnFrame(int frame)
{
	return frame - m_frame >= 2;
}

bool CopyOperation::NeedCopy()
{
	return false;
}