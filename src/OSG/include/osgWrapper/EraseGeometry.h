#ifndef ERASE_GEOMETRY
#define ERASE_GEOMETRY
#include <osg\Geometry>

namespace OSGWrapper
{
	class OSG_EXPORT EraseGeometry : public osg::Geometry
	{
	public:
		EraseGeometry(osg::Array* attribute_array, osg::Array* feedback_array, osg::Array* render_array);
		~EraseGeometry();

		void Exchange();
		void SetArray(osg::Array* coord, osg::Array* normal, osg::Array* color, osg::PrimitiveSet* primitive_set);
	protected:
		virtual void drawImplementation(osg::RenderInfo& renderInfo) const;

	private:
		osg::ref_ptr<osg::Array> m_attribute_array;
		osg::ref_ptr<osg::Array> m_feedback_array;
		osg::ref_ptr<osg::Array> m_render_array;
	};

	class OSG_EXPORT ErasorCallback : public osg::NodeCallback
	{
	public:
		ErasorCallback(EraseGeometry* geom)
			:m_geometry(geom)
		{
		}

		~ErasorCallback(){}

        virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
        {
			if(m_geometry.valid())
				m_geometry->Exchange();

            traverse(node,nv);
        }
	protected:
		osg::ref_ptr<EraseGeometry> m_geometry;
	};
}
#endif // ERASE_GEOMETRY