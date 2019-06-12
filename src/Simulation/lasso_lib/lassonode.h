#pragma once
#include <osgWrapper/AttributeUtilNode.h>
#include "operation.h"

class LassoNode : public OSGWrapper::AttributeUtilNode
{
public:
	LassoNode();
	~LassoNode();

	void Set(osg::Vec3Array* coord_array, osg::FloatArray* flag_array);
	bool OnMouse(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
	void OnFrame();

	void SelectAll();
	void DeselectAll();
protected:
	void ReleaseOperation();
protected:
	int m_frame;
	osg::ref_ptr<Operation> m_operation;
	
	osg::ref_ptr<FeedGeometry> m_feed_geometry;
};