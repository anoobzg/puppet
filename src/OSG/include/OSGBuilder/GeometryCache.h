#pragma once
#include <OSGBuilder/OSGBuilderExport.h>
#include <memory>
#include <osg\Geometry>

namespace OSGBuilder
{
	enum CacheGeometryType
	{
		CGT_Ball,
	};

	class OSG_BUILDER_API GeometryCache
	{
		GeometryCache();
	public:
		~GeometryCache();

		static GeometryCache& Instance();
		osg::Geometry* Get(CacheGeometryType type);
	protected:
		osg::Geometry* GetOrCreateBall();
	private:
		static std::auto_ptr<GeometryCache> m_cache;

		osg::ref_ptr<osg::Geometry> m_ball;
	};
}