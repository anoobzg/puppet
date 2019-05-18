#include "feature_object_traits.h"
#include "feature_object.h"

#include "Mesh.h"

#include <osgWrapper\ArrayCreator.h>
using namespace LauncaGeometry;
osg::Geometry* FeatureObjectTraits::CreateObjectPointCloud(FeatureObject& feature_object, bool down_sample)
{
	std::vector<osg::Vec3f> points;
	std::vector<osg::Vec3f> normals;
	if (down_sample) TraitDownSample(feature_object, points, normals);
	else Trait(feature_object, points, normals);

	return OSGWrapper::GeometryCreator::CreatePointCloud(points, normals);
}

void FeatureObjectTraits::GetPoints(FeatureObject& feature_object, std::vector<unsigned>& indices, std::vector<osg::Vec3f>& points)
{
	if (indices.size() == 0)
		return;

	points.resize(indices.size());
	for (size_t i = 0; i < indices.size(); ++i)
	{
		unsigned idx = indices[i];
		Point& p = feature_object.m_d_cloud->at(idx);
		points[i] = osg::Vec3f(p.x, p.y, p.z);
	}
}

void FeatureObjectTraits::GetBoundary(FeatureObject& object1, FeatureObject& object2, std::vector<osg::Vec3f>& points)
{
	for (size_t i = 0; i < object1.m_inner_index.size(); ++i)
	{
		if (!object1.m_inner_index[i])
		{
			const Point& p = object1.m_d_cloud->at(i);
			points.push_back(osg::Vec3f(p.x, p.y, p.z));
		}
	}

	for (size_t i = 0; i < object2.m_inner_index.size(); ++i)
	{
		if (!object2.m_inner_index[i])
		{
			const Point& p = object2.m_d_cloud->at(i);
			points.push_back(osg::Vec3f(p.x, p.y, p.z));
		}
	}
}

void FeatureObjectTraits::GetISSKeypoints(FeatureObject& object1, FeatureObject& object2, std::vector<osg::Vec3f>& points)
{
	points.reserve(object1.m_iss_key_point->size() + object2.m_iss_key_point->size());
	for (size_t i = 0; i < object1.m_iss_key_point->size(); ++i)
	{
		const Point& p = object1.m_iss_key_point->at(i);
		points.push_back(osg::Vec3f(p.x, p.y, p.z));
	}

	for (size_t i = 0; i < object2.m_iss_key_point->size(); ++i)
	{
		const Point& p = object2.m_iss_key_point->at(i);
		points.push_back(osg::Vec3f(p.x, p.y, p.z));
	}
}

void FeatureObjectTraits::Trait(FeatureObject& feature_object, std::vector<osg::Vec3f>& position, std::vector<osg::Vec3f>& normals)
{
	Mesh& mesh = feature_object.m_mesh;
	if (mesh.vertex_number == 0)
		return;

	position.reserve(mesh.vertex_number);
	normals.reserve(mesh.vertex_number);

	float* p = mesh.vertex_position;
	float* n = mesh.vertex_normal;
	for (unsigned i = 0; i < mesh.vertex_number; ++i)
	{
		position.push_back(osg::Vec3f(*p, *(p + 1), *(p + 2)));
		normals.push_back(osg::Vec3f(*n, *(n + 1), *(n + 2)));
		p += 3;
		n += 3;
	}
}

void FeatureObjectTraits::TraitDownSample(FeatureObject& feature_object, std::vector<osg::Vec3f>& position, std::vector<osg::Vec3f>& normals)
{
	PointCloud::Ptr& point_cloud = feature_object.m_d_cloud;
	NormalCloud::Ptr& normal_cloud = feature_object.m_d_normals;
	if (point_cloud->size() == 0 || normal_cloud->size() == 0)
		return;

	position.reserve(point_cloud->size());
	normals.reserve(normal_cloud->size());

	for (size_t i = 0; i < point_cloud->size(); ++i)
	{
		Point& p = point_cloud->at(i);
		position.push_back(osg::Vec3f(p.x, p.y, p.z));
	}

	for (size_t i = 0; i < point_cloud->size(); ++i)
	{
		Normal& n = normal_cloud->at(i);
		normals.push_back(osg::Vec3f(n.normal_x, n.normal_y, n.normal_z));
	}
}