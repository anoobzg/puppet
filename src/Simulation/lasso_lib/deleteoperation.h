#pragma once
#include "operation.h"

class DeleteOperation : public Operation
{
public:
	DeleteOperation(int frame, FeedGeometry* geometry);
	virtual ~DeleteOperation();

	bool ReleaseOnFrame(int frame);
	const char* OperationName();
protected:
	osg::ref_ptr<FeedGeometry> m_geometry;
	int m_frame;
};