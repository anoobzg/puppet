#pragma once
#include "operation.h"
#include <osg\Texture2D>

class TriangleOperation : public MouseOperation
{
public:
	TriangleOperation(int frame, FeedGeometry* geometry);
	virtual ~TriangleOperation();

	const char* OperationName();
	bool OnMouse(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
	void SetTexture(osg::Texture2D* texture);
protected:
	void UpdateGeometry();
protected:
	osg::ref_ptr<FeedGeometry> m_geometry;
	int m_frame;

	osg::ref_ptr<AttributeUtilNode> m_feed_node;
	osg::ref_ptr<AttributeUtilNode> m_triangle_node;
	osg::ref_ptr<AttributeUtilNode> m_point_node;

	osg::ref_ptr<osg::Vec2Array> m_vertex_array;
	osg::ref_ptr<osg::Geometry> m_points;
	osg::ref_ptr<osg::Geometry> m_triangles;

	osg::ref_ptr<osg::Texture2D> m_texture;
};

