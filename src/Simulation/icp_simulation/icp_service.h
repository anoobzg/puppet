#pragma once
#include "Mesh.h"
#include "GemTransform.h"

#include <pcl\point_types.h>
#include <pcl\point_cloud.h>
#include <pcl\search\kdtree.h>

typedef pcl::PointNormal Point;
typedef pcl::PointCloud<Point> Cloud;
typedef pcl::search::KdTree<Point> KdTree;

using namespace LauncaGeometry;
class ICPService
{
public:
	ICPService(Mesh& stable, Mesh& patch, GemTransform& init_transform);
	~ICPService();

	void Do(GemTransform& transform);
private:
	void BuildPCLCloud(Cloud& cloud, Mesh& mesh);
private:
	Cloud::Ptr m_stable;
	Cloud::Ptr m_patch;
	Eigen::Matrix4f m_init_matrix;
};