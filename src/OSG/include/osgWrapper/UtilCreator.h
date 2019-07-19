#pragma once
#include <osg\Geode>
#include <osg\Geometry>

namespace OSGWrapper
{
	class OSG_EXPORT UtilCreator
	{
	public:
		static osg::Geometry* CreateGrid(const osg::Vec3f& bmin, const osg::Vec3& bmax, const osg::Vec3i& size);
		static osg::Vec2Array* CreateUnitArray();
		static osg::Geometry* CreateUnitQuad();
	};
}

#define CREATE_UNIT_ARRAY OSGWrapper::UtilCreator::CreateUnitArray()
#define CREATE_UNIT_QUAD OSGWrapper::UtilCreator::CreateUnitQuad()