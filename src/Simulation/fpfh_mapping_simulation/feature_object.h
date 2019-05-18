#pragma once
#include <pcl\point_types.h>
#include <pcl\point_cloud.h>
#include <pcl\search\kdtree.h>

typedef pcl::PointXYZI Point;
typedef pcl::Normal Normal;
typedef pcl::FPFHSignature33 FPFH;
typedef pcl::Boundary Boundary;
typedef pcl::PointCloud<Point> PointCloud;
typedef pcl::PointCloud<Normal> NormalCloud;
typedef pcl::search::KdTree<Point> KdTree;
typedef pcl::PointCloud<FPFH> FeatureCloud;
typedef pcl::search::KdTree<FPFH> FeatureTree;
typedef pcl::PointCloud<Boundary> BoundaryCloud;

namespace LauncaGeometry
{
	class Mesh;
}

using namespace LauncaGeometry;
class FeatureObject
{
	friend class FeatureObjectTraits;
	friend class Mapping;
public:
	FeatureObject(Mesh& mesh);
	~FeatureObject();

private:
	void CreatePointCloudAndNormal(unsigned vertex_number, float* position, float* normal, PointCloud& point_cloud, NormalCloud& normal_cloud);
	void DownSample(PointCloud::Ptr& origin_cloud, NormalCloud::Ptr& origin_normal, KdTree::Ptr& kdtree, PointCloud::Ptr& d_cloud, NormalCloud::Ptr& d_normals, PointCloud::Ptr& dd_cloud);
	void CalculateFeature(FeatureCloud::Ptr& feature_cloud, FeatureTree::Ptr& feature_kdtree, PointCloud::Ptr& d_cloud, NormalCloud::Ptr& d_normals, KdTree::Ptr& cloud_kdtree, PointCloud::Ptr& dd_cloud);
	void CalculatePossibleIndex();
	void CalculateISSKeypoint();
private:
	Mesh& m_mesh;

	PointCloud::Ptr m_o_cloud;
	NormalCloud::Ptr m_o_normals;

	PointCloud::Ptr m_d_cloud;
	KdTree::Ptr m_d_tree;
	NormalCloud::Ptr m_d_normals;
	BoundaryCloud::Ptr m_d_boundaries;

	PointCloud::Ptr m_dd_cloud;
	KdTree::Ptr m_dd_tree;
	NormalCloud::Ptr m_dd_normals;

	FeatureCloud::Ptr m_d_features;
	FeatureTree::Ptr m_feature_tree;

	std::vector<bool> m_inner_index;

	PointCloud::Ptr m_iss_key_point;
	std::vector<bool> m_iss_index;
	FeatureTree::Ptr m_iss_feature_tree;
	FeatureTree::IndicesPtr m_iss_tree_index;
};