#pragma once
#include "operation.h"

class SelectAllOperation : public Operation
{
public:
	SelectAllOperation(int frame, FeedGeometry* geometry);
	virtual ~SelectAllOperation();

	bool ReleaseOnFrame(int frame);
	const char* OperationName();
protected:
	osg::ref_ptr<FeedGeometry> m_geometry;
	int m_frame;
};