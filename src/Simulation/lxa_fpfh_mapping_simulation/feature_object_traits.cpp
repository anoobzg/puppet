#include "feature_object_traits.h"
#include "feature_object.h"

#include <osgWrapper\ArrayCreator.h>

osg::Geometry* FeatureObjectTraits::CreateObjectPointCloud(FeatureObject& feature_object)
{
	std::vector<osg::Vec3f> points;
	std::vector<osg::Vec3f> normals;
	Trait(feature_object, points, normals);

	return OSGWrapper::GeometryCreator::CreatePointCloud(points, normals);
}

void FeatureObjectTraits::Trait(FeatureObject& feature_object, std::vector<osg::Vec3f>& position, std::vector<osg::Vec3f>& normals)
{
	pcl::PointCloud<Point>::Ptr cloud = feature_object.m_cloud;

	position.reserve(cloud->size());
	normals.reserve(cloud->size());

	for (unsigned i = 0; i < cloud->size(); ++i)
	{
		Point& p = cloud->at(i);
		position.push_back(osg::Vec3f(p.x, p.y, p.z));
		normals.push_back(osg::Vec3f(p.normal_x, p.normal_y, p.normal_z));
	}
}

void FeatureObjectTraits::GetPoints(FeatureObject& feature_object, const osg::Matrixf& m, std::vector<unsigned>& indices, std::vector<osg::Vec3f>& points)
{
	if (indices.size() == 0)
		return;

	points.resize(indices.size());
	for (size_t i = 0; i < indices.size(); ++i)
	{
		unsigned idx = indices[i];
		Point& p = feature_object.m_cloud->at(idx);
		points[i] = osg::Vec3f(p.x, p.y, p.z) * m;
	}
}
