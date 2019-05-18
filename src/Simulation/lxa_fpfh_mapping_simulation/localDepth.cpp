#include "localDepth.h"

std::vector<float> local_depth2cloud(pcl::PointCloud<pcl::PointNormal>::ConstPtr normalcloud,const float &searchradius)
{
	//创建KdTreeFLANN对象，并把创建的点云设置为输入
	//pcl::KdTreeFLANN<pcl::PointNormal> kdtree;
	//kdtree.setInputCloud(normalcloud);
	//
	std::vector<float> local_depthSet;
	//
	//int search_result = 0;
	//std::vector<int> pointIdxNKNSearch;                           //存储查询点近邻索引
	//std::vector<float> pointNKNSquaredDistance;                   //存储近邻点对应距离平方
	//Eigen::Vector3f project_point(1, 1, 1), PA_point(1, 1, 1), centerPoint_normal(1, 1, 1);
	//float depth_point2projecpoints = 0;
	//for (int ipn=0;ipn<normalcloud->size();ipn++)
	//{
    //    search_result = kdtree.radiusSearch(normalcloud->points[ipn], searchradius, pointIdxNKNSearch, pointNKNSquaredDistance);
	//	if (0!= search_result)
	//	{
	//		//待查询点的主方向和球面的2D切面的交点
	//		PA_point(0) = normalcloud->points[ipn].x + searchradius*normalcloud->points[ipn].normal_x;
	//		PA_point(1) = normalcloud->points[ipn].y + searchradius*normalcloud->points[ipn].normal_y;
	//		PA_point(2) = normalcloud->points[ipn].z + searchradius*normalcloud->points[ipn].normal_z;
	//
	//		//待查询点的坐标,(球心坐标)
	//		centerPoint_normal(0) = normalcloud->points[ipn].normal_x;
	//		centerPoint_normal(1) = normalcloud->points[ipn].normal_y;
	//		centerPoint_normal(2) = normalcloud->points[ipn].normal_z;
	//
	//		//计算球面内每个点到切面的距离
	//		for (auto iv : pointIdxNKNSearch)
	//		{
	//			if (iv != ipn)
	//			{
	//				project_point(0) = normalcloud->points[iv].x;
	//				project_point(1) = normalcloud->points[iv].y;
	//				project_point(2) = normalcloud->points[iv].z;
	//				depth_point2projecpoints = depth_point2projecpoints + distance_point2plance(project_point, PA_point, centerPoint_normal);
	//			}
	//		}
	//		depth_point2projecpoints = depth_point2projecpoints / (pointIdxNKNSearch.size() - 1);
	//
	//		//std::cout << "depth value for point: " << ipn  << "  -------  "<< depth_point2projecpoints << std::endl;
	//
	//		local_depthSet.push_back(depth_point2projecpoints);
	//		depth_point2projecpoints = 0;
	//	}
	//}
	return local_depthSet;
}

//点到平面的距离 pp2PA is point of plance to princple aix
float distance_point2plance(Eigen::Vector3f &point_project,Eigen::Vector3f &pp2PA,Eigen::Vector3f &normal_PA)
{
	float distance = 0;
	distance = abs(normal_PA.dot(pp2PA - point_project));
	return distance;
}