#include "point_source_traits.h"
#include "point_source.h"

void PointSourceTraits::Trait(PointSource& point_source, std::vector<osg::Vec3f>& position, std::vector<osg::Vec3f>& normals, std::vector<float>& flags)
{
	PointCloud::Ptr& point_cloud = point_source.m_point_cloud;
	NormalCloud::Ptr& normal_cloud = point_source.m_normal_cloud;
	if (point_cloud->size() == 0 || normal_cloud->size() == 0)
		return;

	position.reserve(point_cloud->size());
	normals.reserve(normal_cloud->size());
	flags.reserve(point_cloud->size());

	for (size_t i = 0; i < point_cloud->size(); ++i)
	{
		Point& p = point_cloud->at(i);
		position.push_back(osg::Vec3f(p.x, p.y, p.z));
		flags.push_back(1.0f);
	}

	for (size_t i = 0; i < point_cloud->size(); ++i)
	{
		Normal& n = normal_cloud->at(i);
		normals.push_back(osg::Vec3f(n.normal_x, n.normal_y, n.normal_z));
	}
}