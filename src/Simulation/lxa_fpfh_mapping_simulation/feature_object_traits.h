#pragma once
#include <osgWrapper\GeometryCreator.h>

class FeatureObject;
class FeatureObjectTraits
{
public:
	static osg::Geometry* CreateObjectPointCloud(FeatureObject& feature_object);
	static void GetPoints(FeatureObject& feature_object, const osg::Matrixf& m, std::vector<unsigned>& indices, std::vector<osg::Vec3f>& points);
private:
	static void Trait(FeatureObject& feature_object, std::vector<osg::Vec3f>& position, std::vector<osg::Vec3f>& normals);
};
