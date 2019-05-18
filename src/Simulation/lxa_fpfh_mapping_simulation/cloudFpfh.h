#pragma once
#include "common.h"

//计算点云的FPFH,并将计算结果返回
pcl::PointCloud<pcl::FPFHSignature33>::Ptr compute_Cloud_fpfh(pcl::PointCloud<pcl::PointNormal>::ConstPtr normal_cloud,
	const int &searchnum, const float &radius);

//计算点云中所有点的fpfh范数
void compute_fpfhnorm_point(pcl::PointCloud<pcl::FPFHSignature33>::ConstPtr fpfhs_pointcloud, std::vector<float> &norm2fpfhs);


//根据点的FPFH计算点云的KL散度
void compute_fpfhANDcompareKL2Cloud(pcl::PointCloud<pcl::PointNormal>::ConstPtr normalcloud,
	const int &searchnum, const float &radius, const bool &comparefullframe);

//计算两个点FPFH的散度值 fpfh1对fpfh2的相对熵(KL散度)
float KL2cloud_points(pcl::FPFHSignature33 &fpfh1, pcl::FPFHSignature33 &fpfh2);


//计算点云中各点的相关性(相似度),以皮尔逊相关系数度量
void compute_fpfh_correlation2Cloud(pcl::PointCloud<pcl::PointNormal>::ConstPtr normalcloud,
	const int &search_num, const double &searchradius, const bool &comparefullframe);

//计算两个点FPFH的皮尔逊相关系数
float compute_PPMCC_2pointfpfh(pcl::FPFHSignature33 &fpfh1, pcl::FPFHSignature33 &fpfh2);

//计算点云的FPFH，然后根据点云的fpfh计算点云中两点的差别(以向量差的欧氏距离度量)(与全帧比较或与周围临近点比较)
void compute_fpfh_distancediff2Cloud(pcl::PointCloud<pcl::PointNormal>::ConstPtr normalcloud,
	const int &search_num, const double &searchradius, const bool &comparefullframe);

//计算两个点FPFH的欧氏距离
float compute_diff_twopoint_fpfh(pcl::FPFHSignature33 &fpfh1, pcl::FPFHSignature33 &fpfh2);



//搜索固定点附近的点,然后搜索点与固定点比较fpfh值(点数搜索或半径搜索)
void compare_Cloudfpfh(pcl::PointCloud<pcl::PointNormal>::ConstPtr normcloud,
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs,
	const int &fix_index, const int &search_num, const float &searchradius);

/*
//计算点云的FPFH,然后以点云中点为中心进行搜索,再将搜索点与中心点比较fpfh值,如果待比较点与周围点的差别大于搜索数目的2/3
//就可以认为点具有明显的特征
void computeAndcompare_Cloudfpfh(pcl::PointCloud<pcl::PointNormal>::ConstPtr normal_cloud,
	const int search_num, const float scale, const float fpfhdiff_norm_inf);

//计算点云的FPFH,然后以点云中点为中心进行搜索,再将搜索点与中心点比较fpfh值,如果待比较点与周围点的差别大于搜索数目的2/3
//就可以认为点具有明显的特征
void computeAndcompare_Cloudfpfh(pcl::PointCloud<pcl::PointNormal>::ConstPtr normalcloud,
	const int search_num, const float scale);


//计算点云中点的FPFH，然后再计算fpfh的相似度
void computeCloudfpfh_pointsfpfhCorrelation(pcl::PointCloud<pcl::PointNormal>::ConstPtr normalcloud,
	const int searchnum, const float searchradius, const float corr_inf, const float ratio);
	*/

