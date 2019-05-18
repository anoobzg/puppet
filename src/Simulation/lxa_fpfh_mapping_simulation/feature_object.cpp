#include "feature_object.h"
#include <pcl/io/ply_io.h>
#include <pcl\features\fpfh_omp.h>

FeatureObject::FeatureObject(const char* name)
{
	m_cloud.reset(new pcl::PointCloud<Point>());

	Load(*m_cloud, name);

	m_tree.reset(new DKdTree());
	m_tree->setInputCloud(m_cloud);
	m_features.reset(new FeatureCloud());
	m_feature_tree.reset(new FeatureTree());

	typedef pcl::FPFHEstimationOMP<Point, Point, FPFH> FPFHEstimation;
	FPFHEstimation estimation;

	unsigned thread_num = 4;
	float search_radius = 3.0f;
	estimation.setNumberOfThreads(thread_num);
	estimation.setRadiusSearch(search_radius);
	estimation.setInputCloud(m_cloud);
	estimation.setInputNormals(m_cloud);
	estimation.setSearchMethod(m_tree);
	//estimation.setSearchSurface(d_cloud);
	estimation.compute(*m_features);

	m_feature_tree->setInputCloud(m_features);
}

FeatureObject::~FeatureObject()
{

}

void FeatureObject::Load(pcl::PointCloud<Point>& cloud, const char* name)
{
	if (pcl::io::loadPLYFile<pcl::PointNormal>(name, cloud) == -1)
	{
		std::cout << "file load error." << std::endl;
		return;
	}
	return;
}

void FeatureObject::Transform(float x, float y, float z)
{
	for (size_t i = 0; i < m_cloud->size(); ++i)
	{
		Point& p = m_cloud->at(i);
		p.x += x; p.y += y; p.z += z;
	}
}

bool FeatureObject::Valid()
{
	return m_cloud->size() > 0;
}