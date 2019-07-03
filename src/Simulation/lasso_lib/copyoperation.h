#pragma once
#include "operation.h"

class CopyOperation : public Operation
{
public:
	CopyOperation(int frame, FeedGeometry* geometry);
	virtual ~CopyOperation();

	bool ReleaseOnFrame(int frame);
	const char* OperationName();
	bool NeedCopy();
protected:
	osg::ref_ptr<FeedGeometry> m_geometry;
	int m_frame;
};
