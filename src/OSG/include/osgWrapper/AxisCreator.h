#pragma once
#include <osgWrapper\OSGWrapperExporter.h>
#include <osg\Geometry>

namespace OSGWrapper
{
	struct CreateConfig
	{
		bool x;
		bool y;
		bool z;
		float scale;
		float delta;

		CreateConfig()
		{
			x = true;
			y = true;
			z = true;
			scale = 1.4f;
			delta = 0.02f;
		}
	};

	class OSG_WRAPPER_API AxisCreator
	{
	public:
		static osg::Geometry* Create(const osg::BoundingSphere& bounding, const CreateConfig& config = CreateConfig());
		static osg::Geometry* Create(const osg::Vec3f& center, float radius, const CreateConfig& config = CreateConfig());
	};
}