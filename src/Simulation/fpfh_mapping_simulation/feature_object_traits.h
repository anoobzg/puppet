#pragma once
#include <osgWrapper\GeometryCreator.h>

class FeatureObject;
class FeatureObjectTraits
{
public:
	static osg::Geometry* CreateObjectPointCloud(FeatureObject& feature_object, bool down_sample = true);
	static void GetPoints(FeatureObject& feature_object, std::vector<unsigned>& indices, std::vector<osg::Vec3f>& points);
	static void GetBoundary(FeatureObject& object1, FeatureObject& object2, std::vector<osg::Vec3f>& points);
	static void GetISSKeypoints(FeatureObject& object1, FeatureObject& object2, std::vector<osg::Vec3f>& points);
private:
	static void Trait(FeatureObject& feature_object, std::vector<osg::Vec3f>& position, std::vector<osg::Vec3f>& normals);
	static void TraitDownSample(FeatureObject& feature_object, std::vector<osg::Vec3f>& position, std::vector<osg::Vec3f>& normals);
};
