#pragma once
#include <pcl\point_types.h>
#include <pcl\point_cloud.h>
#include <pcl\search\kdtree.h>

typedef pcl::PointXYZI Point;
typedef pcl::Normal Normal;
typedef pcl::PointCloud<Point> PointCloud;
typedef pcl::PointCloud<Normal> NormalCloud;
typedef pcl::search::KdTree<Point> KdTree;