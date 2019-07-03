#pragma once
#include "operation.h"

class ReserveOperation : public Operation
{
public:
	ReserveOperation(int frame, FeedGeometry* geometry);
	virtual ~ReserveOperation();

	bool ReleaseOnFrame(int frame);
	const char* OperationName();
protected:
	osg::ref_ptr<FeedGeometry> m_geometry;
	int m_frame;
};
