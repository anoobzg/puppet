#pragma once
#include "operation.h"

class QuadEraseOperation : public MouseOperation
{
public:
	QuadEraseOperation(int frame, FeedGeometry* geometry);
	virtual ~QuadEraseOperation();

	const char* OperationName();
	bool OnMouse(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);

protected:
	void SetInvalid();
	void UpdateRange();
protected:
	osg::ref_ptr<FeedGeometry> m_geometry;
	int m_frame;

	osg::ref_ptr<osg::Uniform> m_rectx_range;
	osg::ref_ptr<osg::Uniform> m_recty_range;

	osg::Vec2f m_first_point;
	osg::Vec2f m_last_point;
};
