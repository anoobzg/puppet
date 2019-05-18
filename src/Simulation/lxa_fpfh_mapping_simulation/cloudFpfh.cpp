#include "cloudFpfh.h"
#include "basetools.h"
#include <pcl/features/fpfh.h>

//计算点云的FPFH,并将计算结果fpfhs返回.如果搜索半径为0,就以点数搜索.
pcl::PointCloud<pcl::FPFHSignature33>::Ptr compute_Cloud_fpfh(pcl::PointCloud<pcl::PointNormal>::ConstPtr normalcloud,
	                                                          const int &searchnum, const float &radius)
{
	//创建FPFH估计对象fpfh，并把输入点云集cloud和法线集normals传递给它
	pcl::FPFHEstimation<pcl::PointNormal, pcl::PointNormal> fpfh;
	fpfh.setInputCloud(normalcloud);
	fpfh.setInputNormals(normalcloud);

	//创建一个空的kd树对象tree，并把它传递给FPFH估计对象。//基于已知的输入数据集，建立kdtree
	pcl::search::KdTree<PointNormal>::Ptr kdtree(new pcl::search::KdTree<PointNormal>);
	fpfh.setSearchMethod(kdtree);

	//设置搜索方式(或半径搜索或数目搜索)
	if (0 != radius)
	{
		std::cout << "compute fpfh for cloud, search with a given radiu" << std::endl;
		//使用所有半径在3厘米范围内的邻元素  //注意：此处使用的半径必须要大于估计表面法线时使用的半径!!!
		fpfh.setRadiusSearch(double(radius));		
	}
	else
	{
		std::cout << "compute fpfh for cloud, search with given numbers" << std::endl;
		fpfh.setKSearch(searchnum);		
	}

	// Compute the features then output datasets
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_temp(new pcl::PointCloud<pcl::FPFHSignature33>());
	fpfh.compute(*fpfhs_temp);

	return fpfhs_temp;
}

//--------------------------------------------------------------------------------------------------------------
//      由于KL-散度计算中有对数的计算，而fpfh有0的存在，所以结果会是负无穷。因此，使用KL计算相似度不可以
//--------------------------------------------------------------------------------------------------------------
//计算点云的FPFH，然后根据点云的fpfh计算点间的KL散度(与全帧比较或与周围临近点比较)
void compute_fpfhANDcompareKL2Cloud(pcl::PointCloud<pcl::PointNormal>::ConstPtr normalcloud,
	                            const int &searchnum, const float &radius,const bool &comparefullframe)
{
	//创建FPFH估计对象fpfh，并把输入点云集cloud和法线集normals传递给它
	pcl::FPFHEstimation<pcl::PointNormal, pcl::PointNormal> fpfh;
	fpfh.setInputCloud(normalcloud);
	fpfh.setInputNormals(normalcloud);

	//创建一个空的kd树对象tree，并把它传递给FPFH估计对象。//基于已知的输入数据集，建立kdtree
	pcl::search::KdTree<PointNormal>::Ptr fpfh_Kdtree(new pcl::search::KdTree<PointNormal>);
	fpfh.setSearchMethod(fpfh_Kdtree);

	//设置搜索方式(或半径搜索或数目搜索)
	if (0 != radius)
	{
		//使用所有半径在3厘米范围内的邻元素  //注意：此处使用的半径必须要大于估计表面法线时使用的半径!!!
		fpfh.setRadiusSearch(radius);
	}
	else// (0 != searchnum)
	{
		fpfh.setKSearch(searchnum);
	}

	// Compute the features then output datasets
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_cloud(new pcl::PointCloud<pcl::FPFHSignature33>());
	fpfh.compute(*fpfhs_cloud);

	if (true==comparefullframe)
	{
		for (int ipoint = 0; ipoint < normalcloud->size(); ipoint++)
		{
			std::cout << "Kullback-Leibler Divergence: " << ipoint << std::endl;
			for (int jpoint = ipoint + 1; jpoint < normalcloud->size(); jpoint++)
			{
				float kl1 = KL2cloud_points(fpfhs_cloud->points[ipoint], fpfhs_cloud->points[jpoint]);
				std::cout << kl1 << " ";
			}
			std::cout << std::endl;
		}
	}
	else//比较点云的KL散度(与临近点比较)
	{
		std::vector<int> pointIdxNKNSearch;                           //存储查询点近邻索引
		std::vector<float> pointNKNSquaredDistance;                   //存储近邻点对应距离平方
																	  //设置搜索方式(或半径搜索或数目搜索
		for (int ipoint = 0; ipoint < normalcloud->size(); ipoint++)
		{
			if (0 != radius)
			{
				//使用所有半径在3厘米范围内的邻元素  //注意：此处使用的半径必须要大于估计表面法线时使用的半径!!!
				fpfh_Kdtree->radiusSearch(normalcloud->points[ipoint], radius, pointIdxNKNSearch, pointNKNSquaredDistance);
			}
			else
			{
				fpfh_Kdtree->nearestKSearch(normalcloud->points[ipoint], searchnum, pointIdxNKNSearch, pointNKNSquaredDistance);
			}
			std::cout << "Kullback-Leibler Divergence: " << ipoint << " with " << pointIdxNKNSearch.size() << "and: ";
			for (int jsearch = 1; jsearch < pointIdxNKNSearch.size(); jsearch++)
			{
				float kl1 = KL2cloud_points(fpfhs_cloud->points[ipoint], fpfhs_cloud->points[pointIdxNKNSearch[jsearch]]);
				std::cout << kl1 << " ";
			}
			std::cout << std::endl;
		}
	}
	
}

//计算两个点FPFH的散度值 fpfh1对fpfh2的相对熵(KL散度)
float KL2cloud_points(pcl::FPFHSignature33 &fpfh1, pcl::FPFHSignature33 &fpfh2)
{
	float kltemp = 0;
	for (int ifpfh = 0; ifpfh<33; ifpfh++)
	{
		kltemp = kltemp + fpfh1.histogram[ifpfh] * log(fpfh1.histogram[ifpfh] / fpfh2.histogram[ifpfh]);
	}
	return kltemp;
}


//计算点云的FPFH，然后根据点云的fpfh计算点云中各点的相关性(相似度)(与全帧比较或与周围临近点比较)以皮尔逊相关系数度量
void compute_fpfh_correlation2Cloud(pcl::PointCloud<pcl::PointNormal>::ConstPtr normalcloud,
	 const int &search_num, const double &searchradius, const bool &comparefullframe)
{	
	//创建FPFH估计对象fpfh，并把输入点云集cloud和法线集normals传递给它
	pcl::FPFHEstimation<pcl::PointNormal, pcl::PointNormal> fpfh;
	fpfh.setInputCloud(normalcloud);
	fpfh.setInputNormals(normalcloud);

	//创建一个空的kd树对象tree，并把它传递给FPFH估计对象。//基于已知的输入数据集，建立kdtree
	pcl::search::KdTree<PointNormal>::Ptr fpfh_Kdtree(new pcl::search::KdTree<PointNormal>);
	fpfh.setSearchMethod(fpfh_Kdtree);

	//设置搜索方式(或半径搜索或数目搜索)
	if (0 != searchradius)
	{
		//std::cout << "fpfh.setRadiusSearch(searchradius)" << std::endl;
		//使用所有半径在3厘米范围内的邻元素  //注意：此处使用的半径必须要大于估计表面法线时使用的半径!!!
		fpfh.setRadiusSearch(searchradius);
	}
	else// (0 != searchnum)
	{
		//std::cout << "fpfh.setKSearch(search_num)" << std::endl;
		fpfh.setKSearch(search_num);
	}

	// Compute the features then output datasets
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_cloud(new pcl::PointCloud<pcl::FPFHSignature33>());
	fpfh.compute(*fpfhs_cloud);


	int num_points = normalcloud->size();
	float correlation_value = 0;
	if (true == comparefullframe)
	{
		std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
		for (int ifpfh = 0; ifpfh < num_points; ifpfh++)
		{
			std::cout << "correlation_value: " << ifpfh << "---";
			for (int jfpfh = ifpfh + 1; jfpfh < num_points; jfpfh++)
			{
				correlation_value = compute_PPMCC_2pointfpfh(fpfhs_cloud->points[ifpfh], fpfhs_cloud->points[jfpfh]);
				std::cout << correlation_value << "  ";
			}
			std::cout << std::endl;
		}
	}
	else
	{
		std::cout << "###################################################################" << std::endl;

		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr rgbnormalCloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);

		std::vector<int> pointIdxNKNSearch;                           //存储查询点近邻索引
		std::vector<float> pointNKNSquaredDistance;                   //存储近邻点对应距离平方
		int search_ersult = 0;	//设置搜索方式(或半径搜索或数目搜索
		int count = 0;
		for (int ipoint = 0; ipoint < num_points; ipoint++)
		{
			if (0 != searchradius)
			{
				//std::cout << "fpfh_Kdtree->radiusSearch" << std::endl;
				//使用所有半径在3厘米范围内的邻元素  //注意：此处使用的半径必须要大于估计表面法线时使用的半径!!!
				search_ersult = fpfh_Kdtree->radiusSearch(normalcloud->points[ipoint], searchradius, pointIdxNKNSearch, pointNKNSquaredDistance);
			}
			else
			{
				search_ersult = fpfh_Kdtree->nearestKSearch(normalcloud->points[ipoint], search_num, pointIdxNKNSearch, pointNKNSquaredDistance);
			}
			if (0== search_ersult)
			{
				std::cout << "--------- error----------------" << std::endl;
				break;
			}

			std::cout << "correlation_value: " << ipoint << " with " << pointIdxNKNSearch.size() << " points and:   ";
			for (int jsearch = 1; jsearch < pointIdxNKNSearch.size(); jsearch++)
			{
				correlation_value = compute_PPMCC_2pointfpfh(fpfhs_cloud->points[ipoint], fpfhs_cloud->points[pointIdxNKNSearch[jsearch]]);
//				if(abs(correlation_value)>0.99)
//					count++;
//					std::cout << correlation_value << "  ";
//				}
				if (abs(correlation_value)<0.9)
				{
				  count++;
				 // std::cout << correlation_value << "  ";
				}
				//std::cout << correlation_value << "  ";
			}
			float ratio = float(1.0*count) / (pointIdxNKNSearch.size() - 1);
			std::cout << "ratio: " << float(1.0*count)/ (pointIdxNKNSearch.size()-1) << std::endl;
			count = 0;


				//将normal云转化为rgbnormal云
				pcl::PointXYZRGBNormal p;
				p.x = normalcloud->points[ipoint].x;
				p.y = normalcloud->points[ipoint].y;
				p.z = normalcloud->points[ipoint].z;
				p.normal_x = normalcloud->points[ipoint].normal_x;
				p.normal_y = normalcloud->points[ipoint].normal_y;
				p.normal_z = normalcloud->points[ipoint].normal_z;
				
				if (ratio>0.399)
				{//r=255,g=255,b=255代表点为白色
					p.r = 255;
					p.g = 0;
					p.b = 0;
				}
				else if(ratio<0.399&&ratio>0.199)
				{
					p.r = 0;
					p.g = 255;
					p.b = 0;
				}
				else
				{
					p.r = 0;
					p.g = 0;
					p.b = 255;
				}
				rgbnormalCloud->points.push_back(p);
		}
		//viewcloud_3D(rgbnormalCloud);
	}
}

//计算两个点FPFH的皮尔逊相关系数
float compute_PPMCC_2pointfpfh(pcl::FPFHSignature33 &fpfh1, pcl::FPFHSignature33 &fpfh2)
{
	float innerprodut = 0;
	float fpfh1_sum = 0;
	float fpfh2_sum = 0;
	float fpfh1_squarsum = 0;
	float fpfh2_squarsum = 0;
	for (int ifpfh = 0; ifpfh<33; ifpfh++)
	{
		innerprodut = innerprodut + fpfh1.histogram[ifpfh] * fpfh2.histogram[ifpfh];
		fpfh1_sum = fpfh1_sum + fpfh1.histogram[ifpfh];
		fpfh2_sum = fpfh2_sum + fpfh2.histogram[ifpfh];

		fpfh1_squarsum = fpfh1_squarsum + fpfh1.histogram[ifpfh] * fpfh1.histogram[ifpfh];
		fpfh2_squarsum = fpfh2_squarsum + fpfh2.histogram[ifpfh] * fpfh2.histogram[ifpfh];
	}
	float numerator = 0;
	float denominator = 0;
	numerator = 33 * innerprodut - fpfh1_sum*fpfh2_sum;
	denominator = sqrt(33 * fpfh1_squarsum - fpfh1_sum*fpfh1_sum)*sqrt(33 * fpfh2_squarsum - fpfh2_sum*fpfh2_sum);
	return (numerator / denominator);
}

//计算点云的FPFH，然后根据点云的fpfh计算点云中两点的差别(以向量差的欧氏距离度量)(与全帧比较或与周围临近点比较)
void compute_fpfh_distancediff2Cloud(pcl::PointCloud<pcl::PointNormal>::ConstPtr normalcloud,
	const int &search_num, const double &searchradius, const bool &comparefullframe)
{
	//创建FPFH估计对象fpfh，并把输入点云集cloud和法线集normals传递给它
	pcl::FPFHEstimation<pcl::PointNormal, pcl::PointNormal> fpfh;
	fpfh.setInputCloud(normalcloud);
	fpfh.setInputNormals(normalcloud);

	//创建一个空的kd树对象tree，并把它传递给FPFH估计对象。//基于已知的输入数据集，建立kdtree
	pcl::search::KdTree<PointNormal>::Ptr fpfh_Kdtree(new pcl::search::KdTree<PointNormal>);
	fpfh.setSearchMethod(fpfh_Kdtree);

	//设置搜索方式(或半径搜索或数目搜索)
	if (0 != searchradius)
	{
		//std::cout << "fpfh.setRadiusSearch(searchradius)" << std::endl;
		//使用所有半径在3厘米范围内的邻元素  //注意：此处使用的半径必须要大于估计表面法线时使用的半径!!!
		fpfh.setRadiusSearch(searchradius);
	}
	else// (0 != searchnum)
	{
		//std::cout << "fpfh.setKSearch(search_num)" << std::endl;
		fpfh.setKSearch(search_num);
	}

	// Compute the features then output datasets
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_cloud(new pcl::PointCloud<pcl::FPFHSignature33>());
	fpfh.compute(*fpfhs_cloud);

	int num_points = normalcloud->size();
	float distance_diff_norm = 0;
	if (true == comparefullframe)
	{
		std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
		for (int ifpfh = 0; ifpfh < num_points; ifpfh++)
		{
			std::cout << "distance_diff_norm_value: " << ifpfh << "---";
			for (int jfpfh = ifpfh + 1; jfpfh < num_points; jfpfh++)
			{
				distance_diff_norm = compute_diff_twopoint_fpfh(fpfhs_cloud->points[ifpfh], fpfhs_cloud->points[jfpfh]);
				std::cout << distance_diff_norm << "  ";
			}
			std::cout << std::endl;
		}
	}
	else
	{
		std::cout << "###################################################################" << std::endl;
		std::vector<int> pointIdxNKNSearch;                           //存储查询点近邻索引
		std::vector<float> pointNKNSquaredDistance;                   //存储近邻点对应距离平方
		int search_ersult = 0;										  //设置搜索方式(或半径搜索或数目搜索
		for (int ipoint = 0; ipoint < num_points; ipoint++)
		{
			if (0 != searchradius)
			{
				//std::cout << "fpfh_Kdtree->radiusSearch" << std::endl;
				//使用所有半径在3厘米范围内的邻元素  //注意：此处使用的半径必须要大于估计表面法线时使用的半径!!!
				search_ersult = fpfh_Kdtree->radiusSearch(normalcloud->points[ipoint], searchradius, pointIdxNKNSearch, pointNKNSquaredDistance);
			}
			else
			{
				search_ersult = fpfh_Kdtree->nearestKSearch(normalcloud->points[ipoint], search_num, pointIdxNKNSearch, pointNKNSquaredDistance);
			}
			if (0 == search_ersult)
			{
				std::cout << "--------- error----------------" << std::endl;
				break;
			}

			std::cout << "distance_diff_norm_value: " << ipoint << " with " << pointIdxNKNSearch.size() << " points and:   ";
			for (int jsearch = 1; jsearch < pointIdxNKNSearch.size(); jsearch++)
			{
				distance_diff_norm = compute_diff_twopoint_fpfh(fpfhs_cloud->points[ipoint], fpfhs_cloud->points[pointIdxNKNSearch[jsearch]]);
				
				//std::cout << distance_diff_norm << "  ";
				if (distance_diff_norm<10.01)
				{
					std::cout << distance_diff_norm << "  ";
				}
			}
			std::cout << std::endl;
		}
	}
}

//计算两个点FPFH的欧氏距离
float compute_diff_twopoint_fpfh(pcl::FPFHSignature33 &fpfh1, pcl::FPFHSignature33 &fpfh2)
{
	float sum_squre = 0;
	float sqrt_norm = 0;
	for (int ifpfh = 0; ifpfh < 33; ifpfh++)
	{
		sum_squre= sum_squre+(fpfh1.histogram[ifpfh] - fpfh2.histogram[ifpfh])*(fpfh1.histogram[ifpfh] - fpfh2.histogram[ifpfh]);
	}
	sqrt_norm = sqrt(sum_squre);
	return sqrt_norm;
}

//计算点云中所有点的fpfh范数
void compute_fpfhnorm_point(pcl::PointCloud<pcl::FPFHSignature33>::ConstPtr fpfhs_pointcloud, std::vector<float> &norm2fpfhs)
{
	if (fpfhs_pointcloud->empty())
	{
		return;
	}
	int num_point = fpfhs_pointcloud->size();  //num_point也等于(fpfhs_pointcloud->height)*(fpfhs_pointcloud->width);
	std::cout << "----------number of point for pointcloud-------------: " << num_point << std::endl;
	float temp = 0;
	std::cout << "------------------------------------------------compute the norm of fpfh for point in pointcloud-----------------------------------------" << std::endl;
	for (int ipoint = 0; ipoint < num_point; ipoint++)
	{
		temp = 0;
		for (int ihist = 0; ihist<33; ihist++)
		{
			temp = temp + (fpfhs_pointcloud->points[ipoint].histogram[ihist])* (fpfhs_pointcloud->points[ipoint].histogram[ihist]);
		}
		norm2fpfhs.push_back(temp);
	}

	/*
	for (int iNorm = 0; iNorm<fpfhsNorm.size(); iNorm++)
	{
	std::cout << setiosflags(ios::fixed) << setprecision(5) << fpfhsNorm[iNorm] << "    ";
	if ((iNorm + 1) % 14 == 0)
	{
	std::cout << std::endl;
	}
	}
	std::cout << std::endl;
	*/
}

//搜索固定点附近的点,然后搜索点与固定点比较fpfh值(点数搜索或半径搜索)
void compare_Cloudfpfh(pcl::PointCloud<pcl::PointNormal>::ConstPtr normcloud, 
	                   pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs,
	                   const int &fix_index, const int &search_num,const float &searchradius)
{
	//创建KdTreeFLANN对象，并把创建的点云设置为输入
	pcl::KdTreeFLANN<pcl::PointNormal> kdtreeFpfh;
	kdtreeFpfh.setInputCloud(normcloud);

	std::vector<int> pointIdxNKNSearch;                           //存储查询点近邻索引
	std::vector<float> pointNKNSquaredDistance;                   //存储近邻点对应距离平方
	int search_result = 0;
	if (0 != searchradius)   //如果搜索半径不等于0，就按照半径搜索，否则按照点数搜索
	{
		search_result = kdtreeFpfh.radiusSearch(normcloud->points[fix_index], searchradius, pointIdxNKNSearch, pointNKNSquaredDistance);
	}
	else
	{
		search_result = kdtreeFpfh.nearestKSearch(normcloud->points[fix_index], search_num, pointIdxNKNSearch, pointNKNSquaredDistance);
	}
	if (0 == search_result)
	{
		return;
	}

	std::cout << "K nearest neighbor search at (" << normcloud->points[fix_index].x
		<< " " << normcloud->points[fix_index].y
		<< " " << normcloud->points[fix_index].z
		<< ") with neiborhood:"<<std::endl;

	for (int i = 0; i < pointIdxNKNSearch.size(); ++i)
	{
		std::cout <<i << ":  " << normcloud->points[pointIdxNKNSearch[i]].x
					<< " " << normcloud->points[pointIdxNKNSearch[i]].y
					<< " " << normcloud->points[pointIdxNKNSearch[i]].z
					<< " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
	}

	vector<float>diffSum(0);
	float correlation_value = 0;
	for (int isearch = 1; isearch<pointIdxNKNSearch.size(); isearch++)
	{
		correlation_value = compute_PPMCC_2pointfpfh(fpfhs->points[fix_index], fpfhs->points[isearch]);
	}
}


/*
//计算点云的FPFH,然后以点云中点为中心进行搜索,再将搜索点与中心点比较fpfh值,如果待比较点与周围点的差别大于搜索数目的2/3
//就可以认为点具有明显的特征
void computeAndcompare_Cloudfpfh(pcl::PointCloud<pcl::PointNormal>::ConstPtr normalcloud,
	const int search_num, const float scale, const float fpfhdiff_norm_inf)
{
	//创建FPFH估计对象fpfh，并把输入数据集cloud和法线normals传递给它
	pcl::FPFHEstimation<pcl::PointNormal, pcl::PointNormal> fpfh;
	fpfh.setInputCloud(normalcloud);
	fpfh.setInputNormals(normalcloud);

	//创建一个空的kd树对象kdtree，并把它传递给FPFH估计对象。//基于已知的输入数据集，建立kdtree
	pcl::search::KdTree<PointNormal>::Ptr kdtree(new pcl::search::KdTree<PointNormal>);
	fpfh.setSearchMethod(kdtree);

	//使用所有半径在2厘米范围内的邻元素  //注意：此处使用的半径必须要大于估计表面法线时使用的半径!!!
	fpfh.setRadiusSearch(2);

	// Output datasets
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs(new pcl::PointCloud<pcl::FPFHSignature33>());
	// Compute the features
	fpfh.compute(*fpfhs);

	//将normal云转换为rgbnormal云，用于彩色可视化，特征比较突出的点显示为红色
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr normalrgbcloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	transform_normal2rgbnormal(normalcloud, normalrgbcloud);

	std::vector<int> pointIdxNKNSearch(search_num);         //存储查询点近邻索引
	std::vector<float> pointNKNSquaredDistance(search_num); //存储近邻点对应距离平方

	int mum_points = normalcloud->size();
	for (int ipoint = 0; ipoint<mum_points; ipoint++)
	{
		//		std::cout << "K nearest neighbor search at (" << normal_cloud->points[ipoint].x
		//			<< " " << normal_cloud->points[ipoint].y
		//			<< " " << normal_cloud->points[ipoint].z
		//			<< ") with search_num= " << search_num << std::endl;
		int count = 0;
		if (kdtree->nearestKSearch(normalcloud->points[ipoint], search_num, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
		{
			for (int isearch = 1; isearch<pointIdxNKNSearch.size(); isearch++)
			{
				float fpfh_histogram_diff[33] = { 0 };
				for (int ihist = 0; ihist<33; ihist++)
				{
					fpfh_histogram_diff[ihist] = fpfhs->points[isearch].histogram[ihist] - fpfhs->points[ipoint].histogram[ihist];
				}
				if (fpfhdiff_norm_inf<normSum2array(fpfh_histogram_diff, 33))
				{
					count = count + 1;
				}
			}
		}

		if (int(scale*search_num)<count)
		{
			normalrgbcloud->points[ipoint].r = 255;
			normalrgbcloud->points[ipoint].g = 0;
			normalrgbcloud->points[ipoint].b = 0;
		}
	}

	viewcloud_3D(normalrgbcloud);
}



//计算点云的FPFH,然后以点云中点为中心进行搜索,再将搜索点与中心点比较fpfh值,如果待比较点与周围点的差别大于搜索数目的2/3
//就可以认为点具有明显的特征
void computeAndcompare_Cloudfpfh(pcl::PointCloud<pcl::PointNormal>::ConstPtr normalcloud,
	const int search_num, const float scale)
{
	//创建FPFH估计对象fpfh，并把输入数据集cloud和法线normals传递给它
	pcl::FPFHEstimation<pcl::PointNormal, pcl::PointNormal> fpfh;
	fpfh.setInputCloud(normalcloud);
	fpfh.setInputNormals(normalcloud);

	//创建一个空的kd树对象kdtree，并把它传递给FPFH估计对象。//基于已知的输入数据集，建立kdtree
	pcl::search::KdTree<PointNormal>::Ptr kdtree(new pcl::search::KdTree<PointNormal>);
	fpfh.setSearchMethod(kdtree);

	//使用所有半径在2厘米范围内的邻元素  //注意：此处使用的半径必须要大于估计表面法线时使用的半径!!!
	fpfh.setRadiusSearch(2);

	// Output datasets
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs(new pcl::PointCloud<pcl::FPFHSignature33>());
	// Compute the features
	fpfh.compute(*fpfhs);

	//将normal云转换为rgbnormal云，用于彩色可视化，特征比较突出的点显示为红色
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr normalrgbcloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	transform_normal2rgbnormal(normalcloud, normalrgbcloud);

	std::vector<int> pointIdxNKNSearch(search_num);         //存储查询点近邻索引
	std::vector<float> pointNKNSquaredDistance(search_num); //存储近邻点对应距离平方

	int mum_points = normalcloud->size();
	for (int ipoint = 0; ipoint<mum_points; ipoint++)
	{
		//		std::cout << "K nearest neighbor search at (" << normal_cloud->points[ipoint].x
		//			<< " " << normal_cloud->points[ipoint].y
		//			<< " " << normal_cloud->points[ipoint].z
		//			<< ") with search_num= " << search_num << std::endl;
		float max = -100000.0;
		float min = 1000000.0;
		int count = 0;
		if (kdtree->nearestKSearch(normalcloud->points[ipoint], search_num, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
		{
			vector<float> diff_normalvector(0);
			for (int isearch = 1; isearch<pointIdxNKNSearch.size(); isearch++)
			{
				float fpfh_histogram_diff[33] = { 0 };
				for (int ihist = 0; ihist<33; ihist++)
				{
					fpfh_histogram_diff[ihist] = fpfhs->points[isearch].histogram[ihist] - fpfhs->points[ipoint].histogram[ihist];
				}
				diff_normalvector.push_back(normSum2array(fpfh_histogram_diff, 33));
			}
			max = vector_maxvalue(diff_normalvector);
			min = vector_minvalue(diff_normalvector);
			float standard_value = min + 0.5*(max - min);
			for (int idiff = 0; idiff < diff_normalvector.size(); idiff++)
			{
				if (diff_normalvector[idiff]>standard_value)
				{
					count++;
				}
			}
		}

		if (int(scale*search_num)<count)
		{
			normalrgbcloud->points[ipoint].r = 255;
			normalrgbcloud->points[ipoint].g = 0;
			normalrgbcloud->points[ipoint].b = 0;
		}
	}
	viewcloud_3D(normalrgbcloud);
}


//计算点云中点的FPFH，然后再计算fpfh的相似度 以皮尔逊相关系数度量 normalcloud为输入点云,searchnum为搜索点数目,searchradius为搜索半径
//fpfh的相关值下界corr_inf,ratio为相关点数目的比重下界
void computeCloudfpfh_pointsfpfhCorrelation(pcl::PointCloud<pcl::PointNormal>::ConstPtr normalcloud, const int searchnum,
	const float searchradius, const float corr_inf, const float ratio)
{
	//创建FPFH估计对象fpfh，并把输入数据集cloud和法线normals传递给它
	pcl::FPFHEstimation<pcl::PointNormal, pcl::PointNormal> fpfh;
	fpfh.setInputCloud(normalcloud);
	fpfh.setInputNormals(normalcloud);

	//创建一个空的kd树对象kdtree，并把它传递给FPFH估计对象。//基于已知的输入数据集，建立kdtree
	pcl::search::KdTree<PointNormal>::Ptr kdtree(new pcl::search::KdTree<PointNormal>);
	fpfh.setSearchMethod(kdtree);

	//使用所有半径在2厘米范围内的邻元素  //注意：此处使用的半径必须要大于估计表面法线时使用的半径!!!
	fpfh.setRadiusSearch(2);

	// Output datasets
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs(new pcl::PointCloud<pcl::FPFHSignature33>());
	// Compute the features
	fpfh.compute(*fpfhs);



	//将normal云转换为rgbnormal云，用于彩色可视化，特征比较突出的点显示为红色
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr normalrgbcloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	transform_normal2rgbnormal(normalcloud, normalrgbcloud);

	//---------------------------------计算fpfh的相关关系---------------------
	std::vector<int> pointIdxNKNSearch;         //存储查询点近邻索引
	std::vector<float> pointNKNSquaredDistance; //存储近邻点对应距离平方
	int mum_points = normalcloud->size();
	float corr_value = 0;
	//如果搜索点数不等于0，就以点数搜索。否则以半径搜索
	if (0 != searchnum)
	{
		for (int ipoint = 0; ipoint<mum_points; ipoint++)
		{
			int count_corr = 0;
			//int count_incorr = 0;
			if (kdtree->nearestKSearch(normalcloud->points[ipoint], searchnum, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
			{
				//float diffarray[33] = { 0 };
				for (int isearch = 1; isearch<pointIdxNKNSearch.size(); isearch++)
				{
					//diff2array(fpfhs->points[ipoint].histogram, fpfhs->points[isearch].histogram, diffarray, 33);
					//if (sqrt(normSum2array(diffarray, 33))<45) { count++; };

					corr_value = compute_PPMCC_2pointfpfh(fpfhs->points[ipoint], fpfhs->points[isearch]);
					if (corr_value > corr_inf) { count_corr++; }
					//if (corr_value < 0.21) { count_incorr++; }
				}

				if ((float(count_corr) / (pointIdxNKNSearch.size() - 1))>ratio)
				{
					normalrgbcloud->points[ipoint].r = 255;
					normalrgbcloud->points[ipoint].g = 0;
					normalrgbcloud->points[ipoint].b = 0;
				}

				//if ((float(count_incorr) / (pointIdxNKNSearch.size() - 1))>0.5)
				//{
				//	normalrgbcloud->points[ipoint].r = 0;
				//	normalrgbcloud->points[ipoint].g = 255;
				//	normalrgbcloud->points[ipoint].b = 0;
				//}
			}
		}

	}
	else
	{
		std::cout << "Search with radius" << std::endl;
		for (int ipoint = 0; ipoint < mum_points; ipoint++)
		{
			int count_corr = 0;
			//int count_incorr = 0;
			if (kdtree->radiusSearch(normalcloud->points[ipoint], searchradius, pointIdxNKNSearch, pointNKNSquaredDistance))
			{
				//float diffarray[33] = { 0 };
				for (int isearch = 1; isearch<pointIdxNKNSearch.size(); isearch++)
				{
					corr_value = compute_PPMCC_2pointfpfh(fpfhs->points[ipoint], fpfhs->points[isearch]);
					if (corr_value > corr_inf) { count_corr++; }
					//if (corr_value < 0.51) { count_incorr++; }

					//diff2array(fpfhs->points[ipoint].histogram, fpfhs->points[isearch].histogram,diffarray, 33);
					//if(sqrt(normSum2array(diffarray,33))<45) { count++;};
				}

				if ((float(count_corr) / (pointIdxNKNSearch.size() - 1))>ratio)
				{
					normalrgbcloud->points[ipoint].r = 255;
					normalrgbcloud->points[ipoint].g = 0;
					normalrgbcloud->points[ipoint].b = 0;
				}

				//if ((float(count_incorr) / (pointIdxNKNSearch.size() - 1))>0.5)
				//{
				//	normalrgbcloud->points[ipoint].r = 0;
				//	normalrgbcloud->points[ipoint].g = 255;
				//	normalrgbcloud->points[ipoint].b = 0;
				//}
			}
		}
	}
}
*/



