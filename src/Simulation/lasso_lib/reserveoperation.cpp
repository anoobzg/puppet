#include "reserveoperation.h"
#include <osgWrapper/ProgramManager.h>

ReserveOperation::ReserveOperation(int frame, FeedGeometry* geometry)
	:m_geometry(geometry), m_frame(frame)
{
	SetRenderProgram("feed_reserve");

	AddChild(m_geometry);
}

ReserveOperation::~ReserveOperation()
{

}

const char* ReserveOperation::OperationName()
{
	return "ReserveOperation";
}

bool ReserveOperation::ReleaseOnFrame(int frame)
{
	return frame - m_frame >= 3;
}