#pragma once
#include "common.h"

//求点云没和点的局部深度值
std::vector<float> local_depth2cloud(pcl::PointCloud<pcl::PointNormal>::ConstPtr normalcloud, const float &searchradius);


//点到平面的距离 pp2PA is point of plance to princple aix
float distance_point2plance(Eigen::Vector3f &point_project, Eigen::Vector3f &pp2PA, Eigen::Vector3f &normal_PA);