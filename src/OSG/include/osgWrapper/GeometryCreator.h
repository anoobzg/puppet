#ifndef GEOMETRY_CREATOR_H
#define GEOMETRY_CREATOR_H
#include <osg\Geode>
#include <osg\Geometry>

namespace OSGWrapper
{

class OSG_EXPORT GeometryCreator
{
public:
	static osg::Geometry* CreateIndexAttributeGeometry(osg::PrimitiveSet* primitive, osg::Array* attribute0 = 0, osg::Array* attribute1 = 0,
		osg::Array* attribute2 = 0, osg::Array* attribute3 = 0, osg::Array* attribute4 = 0);

	static osg::Geometry* CreatePointCloud(std::vector<osg::Vec3f>& cloud, std::vector<osg::Vec3f>& normals);
	static osg::Geometry* CreatePointCloud(std::vector<float>& cloud, std::vector<float>& normals);
	static osg::Geometry* CreatePointCloud(float* cloud, float* normals, int count);

	static osg::Geometry* CreateLines(std::vector<osg::Vec3f>& lines);
	static osg::Geometry* CreateLines(std::vector<osg::Vec3f>& points1, std::vector<osg::Vec3f>& points2);
};

}
#endif // GEOMETRY_CREATOR_H