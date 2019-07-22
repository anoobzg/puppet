#pragma once
#include <osgWrapper\UIQuad.h>

class Quad : public OSGWrapper::UIQuad
{
public:
	Quad();
	~Quad();

	void SetOffset(const osg::Vec2f& offset);
	void SetSize(const osg::Vec2f& size);
	void SetRect(const osg::Vec2f& offset, const osg::Vec2f& size);

	OSGWrapper::QuadAttributeUtilNode* Generate();
	OSGWrapper::UIQuad* HitTest(float x, float y);
protected:
	void UpdateGeometry();
private:
	osg::Vec2f m_offset;
	osg::Vec2f m_size;

	osg::ref_ptr<OSGWrapper::QuadAttributeUtilNode> m_node;
	osg::ref_ptr<osg::Geometry> m_geometry;
};