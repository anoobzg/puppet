#pragma once
#include "operation.h"

class DeleteAllOperation : public Operation
{
public:
	DeleteAllOperation(int frame, FeedGeometry* geometry);
	virtual ~DeleteAllOperation();

	bool ReleaseOnFrame(int frame);
	const char* OperationName();
protected:
	osg::ref_ptr<FeedGeometry> m_geometry;
	int m_frame;
};