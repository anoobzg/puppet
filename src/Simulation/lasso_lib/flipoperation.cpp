#include "flipoperation.h"
#include <osgWrapper/ProgramManager.h>

FlipOperation::FlipOperation(int frame, FeedGeometry* geometry)
	:m_geometry(geometry), m_frame(frame)
{
	SetRenderProgram("feed_flip");

	AddChild(m_geometry);
}

FlipOperation::~FlipOperation()
{

}

const char* FlipOperation::OperationName()
{
	return "FlipOperation";
}

bool FlipOperation::ReleaseOnFrame(int frame)
{
	return frame - m_frame >= 2;
}
