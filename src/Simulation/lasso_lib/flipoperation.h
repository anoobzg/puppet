#pragma once
#include "operation.h"

class FlipOperation : public Operation
{
public:
	FlipOperation(int frame, FeedGeometry* geometry);
	virtual ~FlipOperation();

	bool ReleaseOnFrame(int frame);
	const char* OperationName();

protected:
	osg::ref_ptr<FeedGeometry> m_geometry;
	int m_frame;
};

