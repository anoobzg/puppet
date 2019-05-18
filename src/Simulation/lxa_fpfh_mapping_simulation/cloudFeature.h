#pragma once
#include "cloudCurature.h"
#include "cloudFpfh.h"
#include <pcl/registration/correspondence_estimation.h>
using namespace pcl;

namespace CloudFeature
{
	struct RGB
	{
		int r;
		int g;
		int b;
	};

	class cloudFeature
	{
	private:
		int fpfh_search_number,curvature_search_number;
		float fpfh_search_radius,curvature_search_radius;
		float  distance_tol,correlate_tol,curva_tol,curva_scale;
		float shapeIndex_tol;
		bool tolerance_by_distance;

	public:
		void setFpfh_search_num(const int number);
		void setCurvature_search_num(const int number);
		void setFpfh_search_radius(const float radius);
		void setCurvature_search_radius(const float radius);
		void setDistance_tol(const int tolerance);
		void setCorrelation_tol(const float tolerance);
		void setCurvature_tol(const float tolerance);
		void setShapeIndex_tol(const float tolerance);
		void setCurvature_scale2select(const float scale);
		void setIfdistance_measure(const bool distance_measure);

		//输入两个点云，先分别计算他们的fpfh,然后再对比两个点云点之间fpfh的相似度(相关系数度量或距离度量),
		//同时对比两个点云中点的曲率，最后返回相似度很贴近的点对集.
		//以距离度量corre_tolerance 代表距离大能大于这个值，相关系数度量代表相关系数不能小于这个值
		void compareClouds_fpfhAndcurature(pcl::PointCloud<pcl::PointNormal>::ConstPtr sourceCloud,
			                               pcl::PointCloud<pcl::PointNormal>::ConstPtr targetCloud, 
			                               pcl::Correspondences &corre_pair_set);

		//输入两个点云，先分别计算他们的fpfh,然后再对比两个点云点之间fpfh的相似度(相关系数度量或距离度量),
		//同时对比两个点云中点的曲率，最后返回相似度很贴近的点对集.
		//以距离度量corre_tolerance 代表距离大能大于这个值，相关系数度量代表相关系数不能小于这个值
		void cloudFeature::compareClouds_fpfhAndcuraturecolor(pcl::PointCloud<pcl::PointNormal>::ConstPtr sourceCloud,
			                                                  pcl::PointCloud<pcl::PointNormal>::ConstPtr targetCloud,
			                                                  pcl::Correspondences &corre_pair_set);

		void compareClouds_fpfhAndcuratureAndshapeIndex(pcl::PointCloud<pcl::PointNormal>::ConstPtr sourceCloud,
			pcl::PointCloud<pcl::PointNormal>::ConstPtr targetCloud,
			pcl::Correspondences &corre_pair_set);

		//输入两个点云，先分别计算他们的fpfh,然后再对比两个点云点之间fpfh的相似度(相关系数度量或距离度量)，
		//最后返回相似度很贴近的点对集.
		//以距离度量corre_tolerance 代表距离大能大于这个值，相关系数度量代表相关系数不能小于这个值
		void compareClouds_fpfh(pcl::PointCloud<pcl::PointNormal>::Ptr normalcloud1,
			                    pcl::PointCloud<pcl::PointNormal>::Ptr normalcloud2,
			                    pcl::Correspondences &corre_pair_set);

		//输入两个点云，先分别计算他们的curature,然后再对比两个点云点之间curature的相似度(距离度量)，
		//最后返回相似度很贴近的点对集.以距离度量corre_tolerance 代表距离不能大于这个值
		void compareClouds_curaturecolor(pcl::PointCloud<pcl::PointNormal>::ConstPtr normalcloud1,
			                        pcl::PointCloud<pcl::PointNormal>::ConstPtr normalcloud2,
			                        pcl::Correspondences &corre_pair_set);

		//输入两个点云，先分别计算他们的曲度,然后再对比两个点云点之间曲度的相似度(距离度量)，
		//最后返回相似度很贴近的点对集.以距离度量corre_tolerance 代表距离不能大于这个值
		void cloudFeature::compareClouds_curvadness(pcl::PointCloud<pcl::PointNormal>::ConstPtr normalcloud1,
			pcl::PointCloud<pcl::PointNormal>::ConstPtr normalcloud2,
			pcl::Correspondences &corre_pair_set);
	};
}