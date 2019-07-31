#pragma once
#include <osgWrapper\UIQuad.h>

namespace OSGWrapper
{

	class OSG_EXPORT Quad : public OSGWrapper::UIQuad
	{
	public:
		Quad();
		~Quad();

		void SetOffset(const osg::Vec2f& offset);
		void SetSize(const osg::Vec2f& size);
		void SetRect(const osg::Vec2f& offset, const osg::Vec2f& size);
		void SetColor(const osg::Vec4f& color);

		OSGWrapper::QuadAttributeUtilNode* Generate();
		OSGWrapper::UIQuad* HitTest(float x, float y);
		void SetUseTexture(bool use);
		void SetTexture(const char* name);
	protected:
		void UpdateGeometry();
	private:
		osg::Vec2f m_offset;
		osg::Vec2f m_size;

		osg::ref_ptr<OSGWrapper::QuadAttributeUtilNode> m_node;
		osg::ref_ptr<osg::Geometry> m_geometry;

		bool m_use_texture;
	};

}