#include "cloudFeature.h"

namespace CloudFeature
{
	void cloudFeature::setFpfh_search_num(const int number)
	{
		fpfh_search_number = number;
	}

	void cloudFeature::setCurvature_search_num(const int number)
	{
		curvature_search_number = number;
	}

	void cloudFeature::setFpfh_search_radius(const float radius)
	{
		fpfh_search_radius = radius;
	}

	void cloudFeature::setCurvature_search_radius(const float radius)
	{
		curvature_search_radius = radius;
	}

	void cloudFeature::setDistance_tol(const int tolerance)
	{
		distance_tol = tolerance;
	}

	void cloudFeature::setCorrelation_tol(const float tolerance)
	{
		correlate_tol = tolerance;
	}

	void cloudFeature::setCurvature_tol(const float tolerance)
	{
		curva_tol = tolerance;
	}

	void cloudFeature::setShapeIndex_tol(const float tolerance)
	{
		shapeIndex_tol = tolerance;
	}

	void cloudFeature::setCurvature_scale2select(const float scale)
	{
		curva_scale = scale;
	}

	void cloudFeature::setIfdistance_measure(const bool distance_measure)
	{
		tolerance_by_distance = distance_measure;
	}


	//输入两个点云，先分别计算他们的fpfh,然后再对比两个点云点之间fpfh的相似度(相关系数度量或距离度量)，
	//最后返回相似度很贴近的点对集.
	//以距离度量corre_tolerance 代表距离大能大于这个值，相关系数度量代表相关系数不能小于这个值
	void cloudFeature::compareClouds_fpfh(pcl::PointCloud<pcl::PointNormal>::Ptr normalcloud1,
		pcl::PointCloud<pcl::PointNormal>::Ptr normalcloud2,
		pcl::Correspondences &corre_pair_set)
	{
		std::cout << "compare point clouds with  point's fpfh" << std::endl;
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs1(new pcl::PointCloud<pcl::FPFHSignature33>()),
			fpfhs2(new pcl::PointCloud<pcl::FPFHSignature33>());

		fpfhs1 = compute_Cloud_fpfh(normalcloud1, fpfh_search_number, fpfh_search_radius);
		fpfhs2 = compute_Cloud_fpfh(normalcloud2, fpfh_search_number, fpfh_search_radius);

		float corre_result = 0;
		pcl::Correspondence corr_pair;

		if (true == tolerance_by_distance)
		{
			std::cout << "compute_diff_twopoint_fpfh" << std::endl;
			for (int ipoint = 0; ipoint < normalcloud1->size(); ipoint++)
			{
				for (int jpoint = 0; jpoint < normalcloud2->size(); jpoint++)
				{
					corre_result = compute_diff_twopoint_fpfh(fpfhs1->points[ipoint], fpfhs2->points[jpoint]);
					if (abs(corre_result) < distance_tol)
					{
						//std::cout << "corre_result:" << corre_result << std::endl;
						//std::cout << "corr_value:" << corre_result << "           ";
						//std::cout << "ipoint-----jpoint: " << ipoint << "------" << jpoint << std::endl;
						//std::cout << "coordinate for two points: " << std::endl;
						//std::cout << normalcloud1->points[ipoint].x << " " << normalcloud1->points[ipoint].y << " " << normalcloud1->points[ipoint].z << std::endl;
						//std::cout << normalcloud2->points[jpoint].x << " " << normalcloud2->points[jpoint].y << " " << normalcloud2->points[jpoint].z << std::endl;
						corr_pair.index_query = ipoint;
						corr_pair.index_match = jpoint;
						corre_pair_set.push_back(corr_pair);
					}
				}
			}
		}
		else
		{
			std::cout << "compute_PPMCC_2pointfpfh" << std::endl;
			for (int ipoint = 0; ipoint < normalcloud1->size(); ipoint++)
			{
				for (int jpoint = 0; jpoint < normalcloud2->size(); jpoint++)
				{
					corre_result = compute_PPMCC_2pointfpfh(fpfhs1->points[ipoint], fpfhs2->points[jpoint]);
					if (abs(corre_result) > correlate_tol)
					{
						//std::cout << "corre_result:" << corre_result << std::endl;
						//std::cout << "corr_value:" << corre_result << "           ";
						//std::cout << "ipoint-----jpoint: " << ipoint << "------" << jpoint << std::endl;
						//std::cout << "coordinate for two points: " << std::endl;
						//std::cout << normalcloud1->points[ipoint].x << " " << normalcloud1->points[ipoint].y << " " << normalcloud1->points[ipoint].z << std::endl;
						//std::cout << normalcloud2->points[jpoint].x << " " << normalcloud2->points[jpoint].y << " " << normalcloud2->points[jpoint].z << std::endl;
						corr_pair.index_query = ipoint;
						corr_pair.index_match = jpoint;
						corre_pair_set.push_back(corr_pair);
					}
				}
			}
		}
	}


	//输入两个点云，先分别计算他们的curature,然后再对比两个点云点之间curature的相似度(距离度量)，
	//最后返回相似度很贴近的点对集.以距离度量corre_tolerance 代表距离不能大于这个值
	void cloudFeature::compareClouds_curaturecolor(pcl::PointCloud<pcl::PointNormal>::ConstPtr normalcloud1,
		pcl::PointCloud<pcl::PointNormal>::ConstPtr normalcloud2,
		pcl::Correspondences &corre_pair_set)
	{
		std::cout << "compare point clouds with point's curature" << std::endl;
		vector<float> cloud1_CVset, cloud2_CVset;
		cloud1_CVset = compute_pointcloud_curvatures(normalcloud1, curvature_search_number, curvature_search_radius);
		cloud2_CVset = compute_pointcloud_curvatures(normalcloud2, curvature_search_number, curvature_search_radius);

		//求点云曲率的最大值和最小值
		std::vector<float> cloud1_cura_maxAndmin, cloud2_cura_maxAndmin;
		cloud1_cura_maxAndmin = compute_maxAndmin2vector(cloud1_CVset);
		cloud2_cura_maxAndmin = compute_maxAndmin2vector(cloud2_CVset);

		//根据曲率划分点云(以颜色划分)。首先利用最大最小值选出划分界线
		float criteria_clou1_cura_1 = 0, criteria_clou1_cura_2 = 0, criteria_clou1_cura_3 = 0, criteria_clou1_cura_4 = 0;
		float criteria_clou2_cura_1 = 0, criteria_clou2_cura_2 = 0, criteria_clou2_cura_3 = 0, criteria_clou2_cura_4 = 0;

		criteria_clou1_cura_1 = cloud1_cura_maxAndmin[1] + (1 - 0.45)*(cloud1_cura_maxAndmin[0] - cloud1_cura_maxAndmin[1]);
		criteria_clou1_cura_2 = cloud1_cura_maxAndmin[1] + (1 - 0.7)*(cloud1_cura_maxAndmin[0] - cloud1_cura_maxAndmin[1]);
		criteria_clou1_cura_4 = cloud1_cura_maxAndmin[1] + (1 - 0.90)*(cloud1_cura_maxAndmin[0] - cloud1_cura_maxAndmin[1]);

		criteria_clou2_cura_1 = cloud2_cura_maxAndmin[1] + (1 - 0.45)*(cloud2_cura_maxAndmin[0] - cloud2_cura_maxAndmin[1]);
		criteria_clou2_cura_2 = cloud2_cura_maxAndmin[1] + (1 - 0.7)*(cloud2_cura_maxAndmin[0] - cloud2_cura_maxAndmin[1]);
		criteria_clou2_cura_4 = cloud2_cura_maxAndmin[1] + (1 - 0.90)*(cloud2_cura_maxAndmin[0] - cloud2_cura_maxAndmin[1]);

		//根据曲率范围对点云进行着色
		std::vector<RGB> rgbV1, rgbV2;
		RGB rgb1, rgb2;

		//根据曲率范围，对sourcecloud进行颜色标记
		for (int ipoint = 0; ipoint < normalcloud1->size(); ipoint++)
		{
			if ((cloud1_CVset[ipoint] > criteria_clou1_cura_1) || (cloud1_CVset[ipoint] == criteria_clou1_cura_1))
			{
				rgb1.r = 255;
				rgb1.g = 0;
				rgb1.b = 0;
			}
			else if ((cloud1_CVset[ipoint] < criteria_clou1_cura_1) && (cloud1_CVset[ipoint] > criteria_clou1_cura_2 || cloud1_CVset[ipoint] == criteria_clou1_cura_2))
			{
				rgb1.r = 0;
				rgb1.g = 255;
				rgb1.b = 0;
			}
			else if ((cloud1_CVset[ipoint] < criteria_clou1_cura_2) && (cloud1_CVset[ipoint] > criteria_clou1_cura_4 || cloud1_CVset[ipoint] == criteria_clou1_cura_4))
			{
				rgb1.r = 0;
				rgb1.g = 0;
				rgb1.b = 255;
			}
			else
			{
				rgb1.r = 255;
				rgb1.g = 255;
				rgb1.b = 255;
			}
			rgbV1.push_back(rgb1);
		}

		//根据曲率范围，对targetcloud进行颜色标记
		for (int ipoint = 0; ipoint < normalcloud2->size(); ipoint++)
		{
			if ((cloud2_CVset[ipoint] > criteria_clou2_cura_1) || (cloud2_CVset[ipoint] == criteria_clou2_cura_1))
			{
				rgb2.r = 255;
				rgb2.g = 0;
				rgb2.b = 0;
			}
			else if ((cloud2_CVset[ipoint] < criteria_clou2_cura_1) && (cloud2_CVset[ipoint] > criteria_clou2_cura_2 || cloud2_CVset[ipoint] == criteria_clou2_cura_2))
			{
				rgb2.r = 0;
				rgb2.g = 255;
				rgb2.b = 0;
			}
			else if ((cloud2_CVset[ipoint] < criteria_clou2_cura_2) && (cloud2_CVset[ipoint] > criteria_clou2_cura_4 || cloud2_CVset[ipoint] == criteria_clou2_cura_4))
			{
				rgb2.r = 0;
				rgb2.g = 0;
				rgb2.b = 255;
			}
			else
			{
				rgb2.r = 255;
				rgb2.g = 255;
				rgb2.b = 255;
			}
			rgbV2.push_back(rgb2);
		}

		pcl::Correspondence corr_pair;
		for (int ipoint = 0; ipoint < normalcloud1->size(); ipoint++)
		{
			for (int jpoint = 0; jpoint < normalcloud2->size(); jpoint++)
			{
				if ((rgbV1[ipoint].r == rgbV2[jpoint].r) && (rgbV1[ipoint].g == rgbV2[jpoint].g) && (rgbV1[ipoint].b == rgbV2[jpoint].b) &&
					(abs(cloud1_CVset[ipoint] - cloud2_CVset[jpoint]) < curva_tol))
				{
					//std::cout << "corre_result:" << abs(cloud1_CVset[ipoint] - cloud2_CVset[jpoint]) << std::endl;
					//std::cout << "corr_value:" << corre_result << "           ";
					//std::cout << "ipoint-----jpoint: " << ipoint << "------" << jpoint << std::endl;
					//std::cout << "coordinate for two points: " << std::endl;
					//std::cout << normalcloud1->points[ipoint].x << " " << normalcloud1->points[ipoint].y << " " << normalcloud1->points[ipoint].z << std::endl;
					//std::cout << normalcloud2->points[jpoint].x << " " << normalcloud2->points[jpoint].y << " " << normalcloud2->points[jpoint].z << std::endl;
					corr_pair.index_query = ipoint;
					corr_pair.index_match = jpoint;
					corre_pair_set.push_back(corr_pair);
				}
			}
		}
	}


	//输入两个点云，先分别计算他们的曲度,然后再对比两个点云点之间曲度的相似度(距离度量)，
	//最后返回相似度很贴近的点对集.以距离度量corre_tolerance 代表距离不能大于这个值
	void cloudFeature::compareClouds_curvadness(pcl::PointCloud<pcl::PointNormal>::ConstPtr normalcloud1,
		pcl::PointCloud<pcl::PointNormal>::ConstPtr normalcloud2,
		pcl::Correspondences &corre_pair_set)
	{
		std::cout << "compare point clouds with point's curvadness" << std::endl;
		vector<float> cloud1_Curvadness_set, cloud2_Curvadness_set;
		cloud1_Curvadness_set = compute_pointcloud_curvadness(normalcloud1, curvature_search_number, curvature_search_radius);
		cloud2_Curvadness_set = compute_pointcloud_curvadness(normalcloud2, curvature_search_number, curvature_search_radius);

		//求点云曲度的最大值和最小值
		std::vector<float> cloud1_curvadness_maxAndmin, cloud2_curvadness_maxAndmin;
		cloud1_curvadness_maxAndmin = compute_maxAndmin2vector(cloud2_Curvadness_set);
		cloud2_curvadness_maxAndmin = compute_maxAndmin2vector(cloud2_Curvadness_set);

		//根据曲度划分点云(以颜色划分)。首先利用最大最小值选出划分界线
		float criteria_clou1_cura_1 = 0, criteria_clou1_cura_2 = 0, criteria_clou1_cura_3 = 0, criteria_clou1_cura_4 = 0;
		float criteria_clou2_cura_1 = 0, criteria_clou2_cura_2 = 0, criteria_clou2_cura_3 = 0, criteria_clou2_cura_4 = 0;

		criteria_clou1_cura_1 = cloud1_curvadness_maxAndmin[1] + (1 - 0.45)*(cloud1_curvadness_maxAndmin[0] - cloud1_curvadness_maxAndmin[1]);
		criteria_clou1_cura_2 = cloud1_curvadness_maxAndmin[1] + (1 - 0.70)*(cloud1_curvadness_maxAndmin[0] - cloud1_curvadness_maxAndmin[1]);
		criteria_clou1_cura_4 = cloud1_curvadness_maxAndmin[1] + (1 - 0.90)*(cloud1_curvadness_maxAndmin[0] - cloud1_curvadness_maxAndmin[1]);

		criteria_clou2_cura_1 = cloud2_curvadness_maxAndmin[1] + (1 - 0.45)*(cloud2_curvadness_maxAndmin[0] - cloud2_curvadness_maxAndmin[1]);
		criteria_clou2_cura_2 = cloud2_curvadness_maxAndmin[1] + (1 - 0.70)*(cloud2_curvadness_maxAndmin[0] - cloud2_curvadness_maxAndmin[1]);
		criteria_clou2_cura_4 = cloud2_curvadness_maxAndmin[1] + (1 - 0.90)*(cloud2_curvadness_maxAndmin[0] - cloud2_curvadness_maxAndmin[1]);

		//根据曲度范围对点云进行着色
		std::vector<RGB> rgbV1, rgbV2;
		RGB rgb1, rgb2;

		//根据曲度范围，对sourcecloud进行颜色标记
		for (int ipoint = 0; ipoint < normalcloud1->size(); ipoint++)
		{
			if ((cloud1_Curvadness_set[ipoint] > criteria_clou1_cura_1) || (cloud1_Curvadness_set[ipoint] == criteria_clou1_cura_1))
			{
				rgb1.r = 255;
				rgb1.g = 0;
				rgb1.b = 0;
			}
			else if ((cloud1_Curvadness_set[ipoint] < criteria_clou1_cura_1) && (cloud1_Curvadness_set[ipoint] > criteria_clou1_cura_2 || cloud1_Curvadness_set[ipoint] == criteria_clou1_cura_2))
			{
				rgb1.r = 0;
				rgb1.g = 255;
				rgb1.b = 0;
			}
			else if ((cloud1_Curvadness_set[ipoint] < criteria_clou1_cura_2) && (cloud1_Curvadness_set[ipoint] > criteria_clou1_cura_4 || cloud1_Curvadness_set[ipoint] == criteria_clou1_cura_4))
			{
				rgb1.r = 0;
				rgb1.g = 0;
				rgb1.b = 255;
			}
			else
			{
				rgb1.r = 255;
				rgb1.g = 255;
				rgb1.b = 255;
			}
			rgbV1.push_back(rgb1);
		}

		//根据曲度范围，对targetcloud进行颜色标记
		for (int ipoint = 0; ipoint < normalcloud2->size(); ipoint++)
		{
			if ((cloud2_Curvadness_set[ipoint] > criteria_clou2_cura_1) || (cloud2_Curvadness_set[ipoint] == criteria_clou2_cura_1))
			{
				rgb2.r = 255;
				rgb2.g = 0;
				rgb2.b = 0;
			}
			else if ((cloud2_Curvadness_set[ipoint] < criteria_clou2_cura_1) && (cloud2_Curvadness_set[ipoint] > criteria_clou2_cura_2 || cloud2_Curvadness_set[ipoint] == criteria_clou2_cura_2))
			{
				rgb2.r = 0;
				rgb2.g = 255;
				rgb2.b = 0;
			}
			else if ((cloud2_Curvadness_set[ipoint] < criteria_clou2_cura_2) && (cloud2_Curvadness_set[ipoint] > criteria_clou2_cura_4 || cloud2_Curvadness_set[ipoint] == criteria_clou2_cura_4))
			{
				rgb2.r = 0;
				rgb2.g = 0;
				rgb2.b = 255;
			}
			else
			{
				rgb2.r = 255;
				rgb2.g = 255;
				rgb2.b = 255;
			}
			rgbV2.push_back(rgb2);
		}

		pcl::Correspondence corr_pair;
		for (int ipoint = 0; ipoint < normalcloud1->size(); ipoint++)
		{
			for (int jpoint = 0; jpoint < normalcloud2->size(); jpoint++)
			{
				if ((rgbV1[ipoint].r == rgbV2[jpoint].r) && (rgbV1[ipoint].g == rgbV2[jpoint].g) && (rgbV1[ipoint].b == rgbV2[jpoint].b) &&
					(abs(cloud1_Curvadness_set[ipoint] - cloud2_Curvadness_set[jpoint]) < curva_tol))
				{
					corr_pair.index_query = ipoint;
					corr_pair.index_match = jpoint;
					corre_pair_set.push_back(corr_pair);
				}
			}
		}
	}


	//输入两个点云，先分别计算他们的fpfh,然后再对比两个点云点之间fpfh的相似度(相关系数度量或距离度量),
	//同时对比两个点云中点的曲率，最后返回相似度很贴近的点对集.
	//以距离度量corre_tolerance 代表距离大能大于这个值，相关系数度量代表相关系数不能小于这个值
	void cloudFeature::compareClouds_fpfhAndcurature(pcl::PointCloud<pcl::PointNormal>::ConstPtr sourceCloud,
		pcl::PointCloud<pcl::PointNormal>::ConstPtr targetCloud,
		pcl::Correspondences &corre_pair_set)
	{
		std::cout << "compare point clouds with every point's fpfh and curature" << std::endl;
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs1(new pcl::PointCloud<pcl::FPFHSignature33>()),
			fpfhs2(new pcl::PointCloud<pcl::FPFHSignature33>());

		fpfhs1 = compute_Cloud_fpfh(sourceCloud, fpfh_search_number, fpfh_search_radius);
		fpfhs2 = compute_Cloud_fpfh(targetCloud, fpfh_search_number, fpfh_search_radius);

		vector<float> cloud1_CVset, cloud2_CVset;
		cloud1_CVset = compute_pointcloud_curvatures(sourceCloud, curvature_search_number, curvature_search_radius);
		cloud2_CVset = compute_pointcloud_curvatures(targetCloud, curvature_search_number, curvature_search_radius);

		std::vector<float> cloud1_cura_maxAndmin, cloud2_cura_maxAndmin;
		cloud1_cura_maxAndmin = compute_maxAndmin2vector(cloud1_CVset);
		cloud2_cura_maxAndmin = compute_maxAndmin2vector(cloud2_CVset);
		float criteria_clou1_cura_04 = 0, criteria_clou1_cura_08 = 0;
		float criteria_clou2_cura_04 = 0, criteria_clou2_cura_08 = 0;

		criteria_clou1_cura_04 = cloud1_cura_maxAndmin[1] + (1 - 0.4)*(cloud1_cura_maxAndmin[0] - cloud1_cura_maxAndmin[1]);
		criteria_clou1_cura_08 = cloud1_cura_maxAndmin[1] + (1 - curva_scale)*(cloud1_cura_maxAndmin[0] - cloud1_cura_maxAndmin[1]);
		criteria_clou2_cura_04 = cloud2_cura_maxAndmin[1] + (1 - 0.4)*(cloud2_cura_maxAndmin[0] - cloud2_cura_maxAndmin[1]);
		criteria_clou2_cura_08 = cloud2_cura_maxAndmin[1] + (1 - curva_scale)*(cloud2_cura_maxAndmin[0] - cloud2_cura_maxAndmin[1]);

		float corre_result = 0;
		pcl::Correspondence corr_pair;
		if (false == tolerance_by_distance)
		{
			std::cout << "compute_PPMCC_2point's fpfh" << std::endl;
			for (int ipoint = 0; ipoint < sourceCloud->size(); ipoint++)
			{
				for (int jpoint = 0; jpoint < targetCloud->size(); jpoint++)
				{
					corre_result = compute_PPMCC_2pointfpfh(fpfhs1->points[ipoint], fpfhs2->points[jpoint]); //需要归一化吗？
					if ((abs(corre_result) > correlate_tol) && (abs(cloud1_CVset[ipoint] - cloud2_CVset[jpoint]) < curva_tol)
						&& (cloud1_CVset[ipoint] > criteria_clou1_cura_08) && (cloud2_CVset[jpoint] > criteria_clou2_cura_08))
					{
						corr_pair.index_query = ipoint;
						corr_pair.index_match = jpoint;
						corre_pair_set.push_back(corr_pair);
					}
				}
			}
		}
		else
		{
			std::cout << "compute_difference_twopoint’s fpfh" << std::endl;
			for (int ipoint = 0; ipoint < sourceCloud->size(); ipoint++)
			{
				for (int jpoint = 0; jpoint < targetCloud->size(); jpoint++)
				{
					corre_result = compute_diff_twopoint_fpfh(fpfhs1->points[ipoint], fpfhs2->points[jpoint]);
					if ((corre_result < distance_tol) && (abs(cloud1_CVset[ipoint] - cloud2_CVset[jpoint]) < curva_tol)
						&& (cloud1_CVset[ipoint] > criteria_clou1_cura_08) && (cloud2_CVset[jpoint] > criteria_clou2_cura_08))
					{
						corr_pair.index_query = ipoint;
						corr_pair.index_match = jpoint;
						corre_pair_set.push_back(corr_pair);
					}
				}
			}
		}
	}


	//输入两个点云，先分别计算他们的fpfh,然后再对比两个点云点之间fpfh的相似度(相关系数度量或距离度量),
	//同时对比两个点云中点的曲率，最后返回相似度很贴近的点对集.
	//以距离度量corre_tolerance 代表距离大能大于这个值，相关系数度量代表相关系数不能小于这个值
	void cloudFeature::compareClouds_fpfhAndcuraturecolor(pcl::PointCloud<pcl::PointNormal>::ConstPtr sourceCloud,
		pcl::PointCloud<pcl::PointNormal>::ConstPtr targetCloud,
		pcl::Correspondences &corre_pair_set)
	{
		std::cout << "compare point clouds with every point's fpfh and curature" << std::endl;
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs1(new pcl::PointCloud<pcl::FPFHSignature33>()),
			fpfhs2(new pcl::PointCloud<pcl::FPFHSignature33>());

		fpfhs1 = compute_Cloud_fpfh(sourceCloud, fpfh_search_number, fpfh_search_radius);
		fpfhs2 = compute_Cloud_fpfh(targetCloud, fpfh_search_number, fpfh_search_radius);

		//vector<float> cloud1_CVset, cloud2_CVset;
		//cloud1_CVset = compute_pointcloud_curvatures(sourceCloud, curvature_search_number, curvature_search_radius);
		//cloud2_CVset = compute_pointcloud_curvatures(targetCloud, curvature_search_number, curvature_search_radius);

		std::vector<float> cloud1_CVset, cloud2_CVset;
		std::vector<float> shapeIndex_set2cloud1, shapeIndex_set2cloud2;
		compute_pointcloud_curvatureAndshapeindex(sourceCloud, cloud1_CVset, shapeIndex_set2cloud1, curvature_search_number, curvature_search_radius);
		compute_pointcloud_curvatureAndshapeindex(targetCloud, cloud2_CVset, shapeIndex_set2cloud2, curvature_search_number, curvature_search_radius);

		std::vector<float> cloud1_cura_maxAndmin, cloud2_cura_maxAndmin;
		cloud1_cura_maxAndmin = compute_maxAndmin2vector(cloud1_CVset);
		cloud2_cura_maxAndmin = compute_maxAndmin2vector(cloud2_CVset);

		float criteria_clou1_cura_1 = 0, criteria_clou1_cura_2 = 0, criteria_clou1_cura_4 = 0;
		float criteria_clou2_cura_1 = 0, criteria_clou2_cura_2 = 0, criteria_clou2_cura_4 = 0;

		criteria_clou1_cura_1 = cloud1_cura_maxAndmin[1] + (1 - 0.5)*(cloud1_cura_maxAndmin[0] - cloud1_cura_maxAndmin[1]);
		criteria_clou1_cura_2 = cloud1_cura_maxAndmin[1] + (1 - 0.75)*(cloud1_cura_maxAndmin[0] - cloud1_cura_maxAndmin[1]);
		criteria_clou1_cura_4 = cloud1_cura_maxAndmin[1] + (1 - 0.90)*(cloud1_cura_maxAndmin[0] - cloud1_cura_maxAndmin[1]);

		criteria_clou2_cura_1 = cloud2_cura_maxAndmin[1] + (1 - 0.5)*(cloud2_cura_maxAndmin[0] - cloud2_cura_maxAndmin[1]);
		criteria_clou2_cura_2 = cloud2_cura_maxAndmin[1] + (1 - 0.75)*(cloud2_cura_maxAndmin[0] - cloud2_cura_maxAndmin[1]);
		criteria_clou2_cura_4 = cloud2_cura_maxAndmin[1] + (1 - 0.90)*(cloud2_cura_maxAndmin[0] - cloud2_cura_maxAndmin[1]);

		std::vector<RGB> rgbV1, rgbV2;
		RGB rgb1, rgb2;

		//根据曲率范围，对sourcecloud进行颜色标记
		for (int ipoint = 0; ipoint < sourceCloud->size(); ipoint++)
		{
			if ((cloud1_CVset[ipoint] > criteria_clou1_cura_1) || (cloud1_CVset[ipoint] == criteria_clou1_cura_1))
			{
				rgb1.r = 255;
				rgb1.g = 0;
				rgb1.b = 0;
			}
			else if ((cloud1_CVset[ipoint] < criteria_clou1_cura_1) && (cloud1_CVset[ipoint] > criteria_clou1_cura_2 || cloud1_CVset[ipoint] == criteria_clou1_cura_2))
			{
				rgb1.r = 0;
				rgb1.g = 255;
				rgb1.b = 0;
			}
			else if ((cloud1_CVset[ipoint] < criteria_clou1_cura_2) && (cloud1_CVset[ipoint] > criteria_clou1_cura_4 || cloud1_CVset[ipoint] == criteria_clou1_cura_4))
			{
				rgb1.r = 0;
				rgb1.g = 0;
				rgb1.b = 255;
			}
			else
			{
				rgb1.r = 255;
				rgb1.g = 255;
				rgb1.b = 255;
			}
			rgbV1.push_back(rgb1);
		}

		//根据曲率范围，对targetcloud进行颜色标记
		for (int ipoint = 0; ipoint < sourceCloud->size(); ipoint++)
		{
			if ((cloud2_CVset[ipoint] > criteria_clou2_cura_1) || (cloud2_CVset[ipoint] == criteria_clou2_cura_1))
			{
				rgb2.r = 255;
				rgb2.g = 0;
				rgb2.b = 0;
			}
			else if ((cloud2_CVset[ipoint] < criteria_clou2_cura_1) && (cloud2_CVset[ipoint] > criteria_clou2_cura_2 || cloud2_CVset[ipoint] == criteria_clou2_cura_2))
			{
				rgb2.r = 0;
				rgb2.g = 255;
				rgb2.b = 0;
			}
			else if ((cloud2_CVset[ipoint] < criteria_clou2_cura_2) && (cloud2_CVset[ipoint] > criteria_clou2_cura_4 || cloud2_CVset[ipoint] == criteria_clou2_cura_4))
			{
				rgb2.r = 0;
				rgb2.g = 0;
				rgb2.b = 255;
			}
			else
			{
				rgb2.r = 255;
				rgb2.g = 255;
				rgb2.b = 255;
			}
			rgbV2.push_back(rgb2);
		}

		float corre_result = 0;
		pcl::Correspondence corr_pair;
		if (false == tolerance_by_distance)
		{
			std::cout << "compute_PPMCC_2point's fpfh" << std::endl;
			for (int ipoint = 0; ipoint < sourceCloud->size(); ipoint++)
			{
				for (int jpoint = 0; jpoint < targetCloud->size(); jpoint++)
				{
					corre_result = compute_PPMCC_2pointfpfh(fpfhs1->points[ipoint], fpfhs2->points[jpoint]);
					if ((abs(corre_result) > correlate_tol) && (rgbV1[ipoint].r == rgbV2[jpoint].r) && (rgbV1[ipoint].g == rgbV2[jpoint].g) && (rgbV1[ipoint].b == rgbV2[jpoint].b)
						&& (abs(shapeIndex_set2cloud1[ipoint] - shapeIndex_set2cloud2[jpoint]) < shapeIndex_tol) && (abs(cloud1_CVset[ipoint] - cloud2_CVset[jpoint]) < curva_tol))
					{
						corr_pair.index_query = ipoint;
						corr_pair.index_match = jpoint;
						corre_pair_set.push_back(corr_pair);
					}
				}
			}
		}
		else
		{
			std::cout << "compute_difference_twopoint’s fpfh" << std::endl;
			for (int ipoint = 0; ipoint < sourceCloud->size(); ipoint++)
			{
				for (int jpoint = 0; jpoint < targetCloud->size(); jpoint++)
				{
					corre_result = compute_diff_twopoint_fpfh(fpfhs1->points[ipoint], fpfhs2->points[jpoint]);
					if ((corre_result < distance_tol) && (rgbV1[ipoint].r == rgbV2[jpoint].r) && (rgbV1[ipoint].g == rgbV2[jpoint].g) && (rgbV1[ipoint].b == rgbV2[jpoint].b)
						&& (abs(shapeIndex_set2cloud1[ipoint] - shapeIndex_set2cloud2[jpoint]) < shapeIndex_tol) && (abs(cloud1_CVset[ipoint] - cloud2_CVset[jpoint]) < curva_tol))
					{
						corr_pair.index_query = ipoint;
						corr_pair.index_match = jpoint;
						corre_pair_set.push_back(corr_pair);
					}
				}
			}
		}
	}






	//输入两个点云，先分别计算他们的fpfh,然后再对比两个点云点之间fpfh的相似度(相关系数度量或距离度量),
	//同时对比两个点云中点的曲率，最后返回相似度很贴近的点对集.
	//以距离度量corre_tolerance 代表距离大能大于这个值，相关系数度量代表相关系数不能小于这个值
	void cloudFeature::compareClouds_fpfhAndcuratureAndshapeIndex(pcl::PointCloud<pcl::PointNormal>::ConstPtr sourceCloud,
		                                                          pcl::PointCloud<pcl::PointNormal>::ConstPtr targetCloud,
		                                                          pcl::Correspondences &corre_pair_set)
	{
		std::cout << "compare point clouds with every point's fpfh and curature and shapeIndex" << std::endl;
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs1(new pcl::PointCloud<pcl::FPFHSignature33>()),
			fpfhs2(new pcl::PointCloud<pcl::FPFHSignature33>());
		//计算输入点云的fpfh
		fpfhs1 = compute_Cloud_fpfh(sourceCloud, fpfh_search_number, fpfh_search_radius);
		fpfhs2 = compute_Cloud_fpfh(targetCloud, fpfh_search_number, fpfh_search_radius);

		//计算输入点云的曲率和形状指数
		std::vector<float> cloud1_CVset, cloud2_CVset;
		std::vector<float> shapeIndex_set2cloud1, shapeIndex_set2cloud2;
		compute_pointcloud_curvatureAndshapeindex(sourceCloud, cloud1_CVset, shapeIndex_set2cloud1, curvature_search_number, curvature_search_radius);
		compute_pointcloud_curvatureAndshapeindex(targetCloud, cloud2_CVset, shapeIndex_set2cloud2, curvature_search_number, curvature_search_radius);

		//计算曲率的最大最小值,为曲率区间划分提供值
		std::vector<float> cloud1_cura_maxAndmin, cloud2_cura_maxAndmin;
		cloud1_cura_maxAndmin = compute_maxAndmin2vector(cloud1_CVset);
		cloud2_cura_maxAndmin = compute_maxAndmin2vector(cloud2_CVset);

		//把点云按曲率划分，并分别着色
		float criteria_clou1_cura_1 = 0, criteria_clou1_cura_2 = 0, criteria_clou1_cura_4 = 0;
		float criteria_clou2_cura_1 = 0, criteria_clou2_cura_2 = 0, criteria_clou2_cura_4 = 0;

		criteria_clou1_cura_1 = cloud1_cura_maxAndmin[1] + (1 - 0.5)*(cloud1_cura_maxAndmin[0] - cloud1_cura_maxAndmin[1]);
		criteria_clou1_cura_2 = cloud1_cura_maxAndmin[1] + (1 - 0.75)*(cloud1_cura_maxAndmin[0] - cloud1_cura_maxAndmin[1]);
		criteria_clou1_cura_4 = cloud1_cura_maxAndmin[1] + (1 - 0.90)*(cloud1_cura_maxAndmin[0] - cloud1_cura_maxAndmin[1]);

		criteria_clou2_cura_1 = cloud2_cura_maxAndmin[1] + (1 - 0.5)*(cloud2_cura_maxAndmin[0] - cloud2_cura_maxAndmin[1]);
		criteria_clou2_cura_2 = cloud2_cura_maxAndmin[1] + (1 - 0.75)*(cloud2_cura_maxAndmin[0] - cloud2_cura_maxAndmin[1]);
		criteria_clou2_cura_4 = cloud2_cura_maxAndmin[1] + (1 - 0.90)*(cloud2_cura_maxAndmin[0] - cloud2_cura_maxAndmin[1]);

		//把点云按形状指数划分，并分别着色
		std::vector<float> cloud1_SI_maxAndmin, cloud2_SI_maxAndmin;
		cloud1_SI_maxAndmin = compute_maxAndmin2vector(shapeIndex_set2cloud1);
		cloud2_SI_maxAndmin = compute_maxAndmin2vector(shapeIndex_set2cloud2);

		float criteria_cloud1_SI_1 = 0, criteria_cloud1_SI_2 = 0, criteria_cloud1_SI_4 = 0;
		float criteria_cloud2_SI_1 = 0, criteria_cloud2_SI_2 = 0, criteria_cloud2_SI_4 = 0;

		criteria_cloud1_SI_1 = cloud1_SI_maxAndmin[1] + (1 - 0.1)*(cloud1_SI_maxAndmin[0] - cloud1_SI_maxAndmin[1]);
		criteria_cloud1_SI_2 = cloud1_SI_maxAndmin[1] + (1 - 0.35)*(cloud1_SI_maxAndmin[0] - cloud1_SI_maxAndmin[1]);
		criteria_cloud1_SI_4 = cloud1_SI_maxAndmin[1] + (1 - 0.6)*(cloud1_SI_maxAndmin[0] - cloud1_SI_maxAndmin[1]);

		criteria_cloud2_SI_1 = cloud2_SI_maxAndmin[1] + (1 - 0.1)*(cloud2_SI_maxAndmin[0] - cloud2_SI_maxAndmin[1]);
		criteria_cloud2_SI_2 = cloud2_SI_maxAndmin[1] + (1 - 0.35)*(cloud2_SI_maxAndmin[0] - cloud2_SI_maxAndmin[1]);
		criteria_cloud2_SI_4 = cloud2_SI_maxAndmin[1] + (1 - 0.6)*(cloud2_SI_maxAndmin[0] - cloud2_SI_maxAndmin[1]);

		//根据曲率范围，对sourcecloud和targetcloud进行颜色标记
		std::vector<RGB> rgb_curvaV1, rgb_curvaV2;
		RGB rgb1, rgb2;
		for (int ipoint = 0; ipoint < sourceCloud->size(); ipoint++)
		{
			if ((cloud1_CVset[ipoint] > criteria_clou1_cura_1) || (cloud1_CVset[ipoint] == criteria_clou1_cura_1))
			{
				rgb1.r = 255;
				rgb1.g = 0;
				rgb1.b = 0;
			}
			else if ((cloud1_CVset[ipoint] < criteria_clou1_cura_1) && (cloud1_CVset[ipoint] > criteria_clou1_cura_2 || cloud1_CVset[ipoint] == criteria_clou1_cura_2))
			{
				rgb1.r = 0;
				rgb1.g = 255;
				rgb1.b = 0;
			}
			else if ((cloud1_CVset[ipoint] < criteria_clou1_cura_2) && (cloud1_CVset[ipoint] > criteria_clou1_cura_4 || cloud1_CVset[ipoint] == criteria_clou1_cura_4))
			{
				rgb1.r = 0;
				rgb1.g = 0;
				rgb1.b = 255;
			}
			else
			{
				rgb1.r = 255;
				rgb1.g = 255;
				rgb1.b = 255;
			}
			rgb_curvaV1.push_back(rgb1);
		}

		for (int ipoint = 0; ipoint < sourceCloud->size(); ipoint++)
		{
			if ((cloud2_CVset[ipoint] > criteria_clou2_cura_1) || (cloud2_CVset[ipoint] == criteria_clou2_cura_1))
			{
				rgb2.r = 255;
				rgb2.g = 0;
				rgb2.b = 0;
			}
			else if ((cloud2_CVset[ipoint] < criteria_clou2_cura_1) && (cloud2_CVset[ipoint] > criteria_clou2_cura_2 || cloud2_CVset[ipoint] == criteria_clou2_cura_2))
			{
				rgb2.r = 0;
				rgb2.g = 255;
				rgb2.b = 0;
			}
			else if ((cloud2_CVset[ipoint] < criteria_clou2_cura_2) && (cloud2_CVset[ipoint] > criteria_clou2_cura_4 || cloud2_CVset[ipoint] == criteria_clou2_cura_4))
			{
				rgb2.r = 0;
				rgb2.g = 0;
				rgb2.b = 255;
			}
			else
			{
				rgb2.r = 255;
				rgb2.g = 255;
				rgb2.b = 255;
			}
			rgb_curvaV2.push_back(rgb2);
		}

		//根据形状指数范围，对sourcecloud和targetcloud进行颜色标记
		std::vector<RGB> rgb_shapeV1, rgb_shapeV2;
		RGB rgbshape1, rgbshape2;
		for (int ipoint = 0; ipoint < sourceCloud->size(); ipoint++)
		{
			if (shapeIndex_set2cloud1[ipoint] >= criteria_cloud1_SI_1)
			{
				rgbshape1.r = 255;
				rgbshape1.g = 0;
				rgbshape1.b = 0;
			}
			else if (shapeIndex_set2cloud1[ipoint] < criteria_cloud1_SI_1 && shapeIndex_set2cloud1[ipoint] >= criteria_cloud1_SI_2)
			{
				rgbshape1.r = 0;
				rgbshape1.g = 255;
				rgbshape1.b = 0;
			}
			else if (shapeIndex_set2cloud1[ipoint] < criteria_cloud1_SI_2 && shapeIndex_set2cloud1[ipoint] >= criteria_cloud1_SI_4 )
			{
				rgbshape1.r = 0;
				rgbshape1.g = 0;
				rgbshape1.b = 255;
			}
			else
			{
				rgbshape1.r = 255;
				rgbshape1.g = 255;
				rgbshape1.b = 255;
			}
			rgb_shapeV1.push_back(rgb1);
		}

		for (int ipoint = 0; ipoint < sourceCloud->size(); ipoint++)
		{
			if (shapeIndex_set2cloud2[ipoint] >= criteria_cloud2_SI_1)
			{
				rgbshape2.r = 255;
				rgbshape2.g = 0;
				rgbshape2.b = 0;
			}
			else if (shapeIndex_set2cloud2[ipoint] < criteria_cloud2_SI_1 && shapeIndex_set2cloud2[ipoint] >= criteria_cloud2_SI_2)
			{
				rgbshape2.r = 0;
				rgbshape2.g = 255;
				rgbshape2.b = 0;
			}
			else if (shapeIndex_set2cloud2[ipoint] < criteria_cloud2_SI_2 && shapeIndex_set2cloud2[ipoint] >= criteria_cloud2_SI_4)
			{
				rgbshape2.r = 0;
				rgbshape2.g = 0;
				rgbshape2.b = 255;
			}
			else
			{
				rgbshape2.r = 255;
				rgbshape2.g = 255;
				rgbshape2.b = 255;
			}
			rgb_shapeV2.push_back(rgb2);
		}

		//根据度量进行点对筛选
		float corre_result = 0;
		pcl::Correspondence corr_pair;
		pcl::Correspondences corre_pair_curvaset, corre_pair_shapeIndexset;
		if (false == tolerance_by_distance)
		{
			std::cout << "compare the curvature" << std::endl;
			for (int ipoint = 0; ipoint < sourceCloud->size(); ipoint++)
			{
				for (int jpoint = 0; jpoint < targetCloud->size(); jpoint++)
				{
					if ((rgb_curvaV1[ipoint].r == rgb_curvaV2[jpoint].r) && (rgb_curvaV1[ipoint].g == rgb_curvaV2[jpoint].g) && (rgb_curvaV1[ipoint].b == rgb_curvaV2[jpoint].b)
						&& (rgb_shapeV1[ipoint].r == rgb_shapeV2[jpoint].r) && (rgb_shapeV1[ipoint].g == rgb_shapeV2[jpoint].g) && (rgb_shapeV1[ipoint].b == rgb_shapeV2[jpoint].b) 
						&& (abs(cloud1_CVset[ipoint] - cloud2_CVset[jpoint]) < curva_tol))
					{
						corr_pair.index_query = ipoint;
						corr_pair.index_match = jpoint;
						corre_pair_curvaset.push_back(corr_pair);
					}
				}
			}

			std::cout << "compare the shape index" << std::endl;
			for (auto c : corre_pair_curvaset)
			{
				if (abs(shapeIndex_set2cloud1[c.index_query] - shapeIndex_set2cloud2[c.index_match]) < shapeIndex_tol)
				{
					corr_pair.index_query = c.index_query;
					corr_pair.index_match = c.index_match;
					corre_pair_shapeIndexset.push_back(corr_pair);
				}
			}


			std::cout << "compare the fpfh" << std::endl;
			for (auto c : corre_pair_shapeIndexset)
			{
				corre_result = compute_PPMCC_2pointfpfh(fpfhs1->points[c.index_query], fpfhs2->points[c.index_match]);
				if (abs(corre_result) > correlate_tol)
				{
					corr_pair.index_query = c.index_query;
					corr_pair.index_match = c.index_match;
					corre_pair_set.push_back(corr_pair);
				}
			}

		}
		else
		{
			std::cout << "compare the curvature" << std::endl;
			for (int ipoint = 0; ipoint < sourceCloud->size(); ipoint++)
			{
				for (int jpoint = 0; jpoint < targetCloud->size(); jpoint++)
				{
					if ((rgb_curvaV1[ipoint].r == rgb_curvaV2[jpoint].r) && (rgb_curvaV1[ipoint].g == rgb_curvaV2[jpoint].g) && (rgb_curvaV1[ipoint].b == rgb_curvaV2[jpoint].b)
						&& (rgb_shapeV1[ipoint].r == rgb_shapeV2[jpoint].r) && (rgb_shapeV1[ipoint].g == rgb_shapeV2[jpoint].g) && (rgb_shapeV1[ipoint].b == rgb_shapeV2[jpoint].b)
						&& (abs(cloud1_CVset[ipoint] - cloud2_CVset[jpoint]) < curva_tol))
					{
						corr_pair.index_query = ipoint;
						corr_pair.index_match = jpoint;
						corre_pair_curvaset.push_back(corr_pair);
					}
				}
			}

			std::cout << "compare the shape index" << std::endl;
			for (auto c : corre_pair_curvaset)
			{
				if (abs(shapeIndex_set2cloud1[c.index_query] - shapeIndex_set2cloud2[c.index_match]) < shapeIndex_tol)
				{
					corr_pair.index_query = c.index_query;
					corr_pair.index_match = c.index_match;
					corre_pair_shapeIndexset.push_back(corr_pair);
				}
			}


			std::cout << "compare the fpfh" << std::endl;
			for (auto c : corre_pair_shapeIndexset)
			{
				corre_result = compute_diff_twopoint_fpfh(fpfhs1->points[c.index_query], fpfhs2->points[c.index_match]);
				if (abs(corre_result) > correlate_tol)
				{
					corr_pair.index_query = c.index_query;
					corr_pair.index_match = c.index_match;
					corre_pair_set.push_back(corr_pair);
				}
			}
		}
	}

}

