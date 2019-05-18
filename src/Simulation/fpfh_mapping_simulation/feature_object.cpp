#include "feature_object.h"
#include <pcl\features\normal_3d_omp.h>
#include <pcl\features\fpfh_omp.h>
#include <pcl\features\boundary.h>

#include <pcl\keypoints\iss_3d.h>
#include <pcl\keypoints\harris_3d.h>

#include "Mesh.h"

#include "user_define_voxel_grid.h"
FeatureObject::FeatureObject(Mesh& mesh)
	:m_mesh(mesh)
{
	m_o_cloud.reset(new PointCloud());
	m_o_normals.reset(new NormalCloud());

	CreatePointCloudAndNormal(m_mesh.vertex_number, m_mesh.vertex_position, m_mesh.vertex_normal, *m_o_cloud, *m_o_normals);

	m_d_cloud.reset(new PointCloud());
	m_d_normals.reset(new NormalCloud());
	m_d_tree.reset(new KdTree());
	m_dd_cloud.reset(new PointCloud());
	DownSample(m_o_cloud, m_o_normals, m_d_tree, m_d_cloud, m_d_normals, m_dd_cloud);

	m_d_features.reset(new FeatureCloud());
	m_feature_tree.reset(new FeatureTree());
	CalculateFeature(m_d_features, m_feature_tree, m_d_cloud, m_d_normals, m_d_tree, m_dd_cloud);

	CalculatePossibleIndex();
	CalculateISSKeypoint();
}

FeatureObject::~FeatureObject()
{

}

void FeatureObject::CreatePointCloudAndNormal(unsigned vertex_number, float* position, float* normal, PointCloud& point_cloud, NormalCloud& normal_cloud)
{
	point_cloud.width = vertex_number;
	point_cloud.height = 1;
	normal_cloud.width = vertex_number;
	normal_cloud.height = 1;

	float* p = position;
	float* n = normal;

	if (vertex_number > 0)
	{
		point_cloud.resize(vertex_number);
		normal_cloud.resize(vertex_number);

		for (unsigned i = 0; i < vertex_number; ++i)
		{
			Point& pp = point_cloud.at(i);
			Normal& nn = normal_cloud.at(i);

			pp.x = *p++; pp.y = *p++; pp.z = *p++;
			nn.normal_x = *n++; nn.normal_y = *n++; nn.normal_z = *n++;
		}
	}
}

void FeatureObject::DownSample(PointCloud::Ptr& origin_cloud, NormalCloud::Ptr& origin_normal, KdTree::Ptr& kdtree, PointCloud::Ptr& d_cloud, NormalCloud::Ptr& d_normals, PointCloud::Ptr& dd_cloud)
{
	//typedef pcl::VoxelGrid<Point> VoxelGrid;
	//VoxelGrid filter;
	typedef UserVoxelGrid<Point, Normal> UserVoxelGrid;
	UserVoxelGrid filter;

	float leaf_size = 0.4f;
	filter.setLeafSize(leaf_size, leaf_size, leaf_size);

	filter.setInputCloud(origin_cloud);
	filter.setInNormalCloud(origin_normal);
	filter.setOutNormalCloud(d_normals);
	filter.filter(*d_cloud);

	kdtree->setInputCloud(d_cloud);
	//typedef pcl::NormalEstimationOMP<Point, Normal> NormalEstimationOMP;
	//NormalEstimationOMP normal_estimation;
	//
	double search_radius = 1.0f;
	//unsigned thread_num = 4;
	//normal_estimation.setInputCloud(d_cloud);
	//normal_estimation.setNumberOfThreads(thread_num);
	//normal_estimation.setRadiusSearch(search_radius);
	//normal_estimation.setSearchMethod(kdtree);
	//normal_estimation.compute(*d_normals);

	m_d_boundaries.reset(new BoundaryCloud());
	pcl::BoundaryEstimation<Point, Normal, Boundary> bestimation;
	bestimation.setInputCloud(m_d_cloud);
	bestimation.setInputNormals(m_d_normals);
	bestimation.setSearchMethod(m_d_tree);
	bestimation.setRadiusSearch(search_radius);
	bestimation.compute(*m_d_boundaries);

	//float d_leaf_size = 0.4f;
	//filter.setLeafSize(d_leaf_size, d_leaf_size, d_leaf_size);
	//filter.filter(*dd_cloud);
	//
	//m_dd_tree.reset(new KdTree());
	//m_dd_tree->setInputCloud(m_dd_cloud);
}

void FeatureObject::CalculateFeature(FeatureCloud::Ptr& feature_cloud, FeatureTree::Ptr& feature_kdtree, PointCloud::Ptr& d_cloud, NormalCloud::Ptr& d_normals, KdTree::Ptr& cloud_kdtree, PointCloud::Ptr& dd_cloud)
{
	typedef pcl::FPFHEstimationOMP<Point, Normal, FPFH> FPFHEstimation;
	FPFHEstimation estimation;

	unsigned thread_num = 4;
	float search_radius = 3.0f;
	estimation.setNumberOfThreads(thread_num);
	estimation.setRadiusSearch(search_radius);
	estimation.setInputCloud(d_cloud);
	estimation.setInputNormals(d_normals);
	estimation.setSearchMethod(cloud_kdtree);
	//estimation.setSearchSurface(d_cloud);
	estimation.compute(*feature_cloud);

	feature_kdtree->setInputCloud(feature_cloud);
}

void FeatureObject::CalculatePossibleIndex()
{
	double radius = 2.0;
	m_inner_index.resize(m_d_cloud->size(), true);

	for (size_t i = 0; i < m_d_boundaries->size(); ++i)
	{
		if (m_d_boundaries->at(i).boundary_point == 1)
		{
			std::vector<int> indices;
			std::vector<float> distances;
			const Point& p = m_d_cloud->at(i);
			m_d_tree->radiusSearch(p, radius, indices, distances);

			for (size_t j = 0; j < indices.size(); ++j)
			{
				m_inner_index[indices[j]] = false;
			}
		}
	}
}

void FeatureObject::CalculateISSKeypoint()
{
	m_iss_key_point.reset(new PointCloud());

	//ISS
	pcl::ISSKeypoint3D<Point, Point> iss_detector;
	iss_detector.setSearchMethod(m_d_tree);
	iss_detector.setSalientRadius(6 * 0.6f);
	iss_detector.setNonMaxRadius(4 * 0.6f);
	
	iss_detector.setThreshold21(0.975);
	iss_detector.setThreshold32(0.975);
	iss_detector.setMinNeighbors(5);
	iss_detector.setNumberOfThreads(4);
	iss_detector.setInputCloud(m_d_cloud);
	iss_detector.compute(*m_iss_key_point);
	//Harris
	//pcl::HarrisKeypoint3D<Point, Point, Normal> harris_detector;
	//harris_detector.setInputCloud(m_d_cloud);
	//harris_detector.setRadius(0.4f);
	//harris_detector.setThreshold(0.02f);
	//harris_detector.compute(*m_iss_key_point);

	m_iss_index.resize(m_d_cloud->size(), false);
	double iss_radius = 1.0;
	for (size_t i = 0; i < m_iss_key_point->size(); ++i)
	{
		const Point& p = m_iss_key_point->at(i);

		std::vector<int> indices;
		std::vector<float> distances;

		m_d_tree->radiusSearch(p, iss_radius, indices, distances);
		for (size_t j = 0; j < indices.size(); ++j)
		{
			int index = indices[j];
			if (m_inner_index[index])
			{
				m_iss_index[index] = true;
			}
		}
	}

	m_iss_feature_tree.reset(new FeatureTree());
	m_iss_tree_index.reset(new std::vector<int>());
	for (size_t i = 0; i < m_iss_index.size(); ++i)
		if (m_iss_index[i])
			m_iss_tree_index->push_back((int)i);
	m_iss_feature_tree->setInputCloud(m_d_features, m_iss_tree_index);
}