#pragma once
#include "operation.h"

class EraseOperation : public MouseOperation
{
public:
	EraseOperation(int frame, FeedGeometry* geometry);
	virtual ~EraseOperation();

	const char* OperationName();
	bool OnMouse(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
protected:
	osg::ref_ptr<FeedGeometry> m_geometry;
	int m_frame;

	osg::ref_ptr<osg::Uniform> m_pickxy;
};