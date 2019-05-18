#pragma once
#include <pcl\point_types.h>
#include <pcl\point_cloud.h>
#include <pcl\search\kdtree.h>

typedef pcl::PointNormal Point;
//typedef pcl::PointCloud<Point> PointCloud;
typedef pcl::FPFHSignature33 FPFH;
typedef pcl::Boundary Boundary;
typedef pcl::search::KdTree<Point> DKdTree;
typedef pcl::PointCloud<FPFH> FeatureCloud;
typedef pcl::search::KdTree<FPFH> FeatureTree;
class FeatureObject
{
	friend class FeatureObjectTraits;
public:
	FeatureObject(const char* name);
	~FeatureObject();

	bool Valid();
	void Transform(float x, float y, float z);
private:
	void Load(pcl::PointCloud<Point>& cloud, const char* name);
public:
	pcl::PointCloud<Point>::Ptr m_cloud;

	DKdTree::Ptr m_tree;
	FeatureCloud::Ptr m_features;
	FeatureTree::Ptr m_feature_tree;
};