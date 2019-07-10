#include <osgWrapper\AxisCreator.h>
#include <osgWrapper\GeometryCreator.h>

namespace OSGWrapper
{
	osg::Geometry* AxisCreator::Create(const osg::BoundingSphere& bounding, const CreateConfig& config)
	{
		osg::Vec3f center = bounding.center();
		float r = bounding.radius();
		return Create(center, r);
	}

	osg::Geometry* AxisCreator::Create(const osg::Vec3f& center, float radius, const CreateConfig& config)
	{
		osg::Vec3Array* coord_array = new osg::Vec3Array();
		osg::Vec4Array* color_array = new osg::Vec4Array();
		float r = config.scale * radius;
		float d = config.delta * r;
		if(config.x)
		{//X
			osg::Vec4f x_color = osg::Vec4f(1.0f, 0.0f, 0.0f, 1.0f);
			osg::Vec3f arrow = center + osg::Vec3f(r, 0.0f, 0.0f);
			coord_array->push_back(center);
			coord_array->push_back(arrow);
			coord_array->push_back(arrow);
			coord_array->push_back(arrow + osg::Vec3f(-d, d, 0.0f));
			coord_array->push_back(arrow);
			coord_array->push_back(arrow + osg::Vec3f(-d, -d, 0.0f));
			coord_array->push_back(arrow + osg::Vec3f(d, d, 0.0f));
			coord_array->push_back(arrow + osg::Vec3f(2.0f * d, -d, 0.0f));
			coord_array->push_back(arrow + osg::Vec3f(2.0f * d, d, 0.0f));
			coord_array->push_back(arrow + osg::Vec3f(d, -d, 0.0f));
			color_array->resize(coord_array->size(), x_color);
		}
		if (config.y)
		{//Y
			osg::Vec4f y_color = osg::Vec4f(0.0f, 1.0f, 0.0f, 1.0f);
			osg::Vec3f arrow = center + osg::Vec3f(0.0f, -r, 0.0f);
			coord_array->push_back(center);
			coord_array->push_back(arrow);
			coord_array->push_back(arrow);
			coord_array->push_back(arrow + osg::Vec3f(-d, d, 0.0f));
			coord_array->push_back(arrow);
			coord_array->push_back(arrow + osg::Vec3f(d, d, 0.0f));
			coord_array->push_back(arrow + osg::Vec3f(0.0f, -d, 0.0f));
			coord_array->push_back(arrow + osg::Vec3f(0.0f, -2.0f * d, 0.0f));
			coord_array->push_back(arrow + osg::Vec3f(0.0f, -2.0f * d, 0.0f));
			coord_array->push_back(arrow + osg::Vec3f(d, -3.0f * d, 0.0f));
			coord_array->push_back(arrow + osg::Vec3f(0.0f, -2.0f * d, 0.0f));
			coord_array->push_back(arrow + osg::Vec3f(-d, -3.0f * d, 0.0f));
			color_array->resize(coord_array->size(), y_color);
		}
		if (config.z)
		{
			osg::Vec4f z_color = osg::Vec4f(0.0f, 0.0f, 1.0f, 1.0f);
			osg::Vec3f arrow = center + osg::Vec3f(0.0f, 0.0f, r);
			coord_array->push_back(center);
			coord_array->push_back(arrow);
			coord_array->push_back(arrow);
			coord_array->push_back(arrow + osg::Vec3f(0.0f, d, -d));
			coord_array->push_back(arrow);
			coord_array->push_back(arrow + osg::Vec3f(0.0f, -d, -d));
			coord_array->push_back(arrow + osg::Vec3f(0.0f, d, d));
			coord_array->push_back(arrow + osg::Vec3f(0.0f, d, 2.0f * d));
			coord_array->push_back(arrow + osg::Vec3f(0.0f, d, 2.0f * d));
			coord_array->push_back(arrow + osg::Vec3f(0.0f, -d, d));
			coord_array->push_back(arrow + osg::Vec3f(0.0f, -d, d));
			coord_array->push_back(arrow + osg::Vec3f(0.0f, -d, 2.0f * d));
			color_array->resize(coord_array->size(), z_color);
		}
		osg::DrawArrays* draw_array = new osg::DrawArrays(GL_LINES, 0, coord_array->size());
		osg::Geometry* geometry = OSGWrapper::GeometryCreator::CreateIndexAttributeGeometry(draw_array, coord_array,
			color_array);
		return geometry;
	}
}