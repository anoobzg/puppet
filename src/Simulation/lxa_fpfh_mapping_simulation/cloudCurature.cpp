#include"cloudCurature.h"
#include <pcl\search\kdtree.h>
#include <pcl/features/principal_curvatures.h>

#include "pointCloudbase.h"
//求点云中每个点的曲率(点云曲率)
std::vector<float> compute_pointcloud_curvatures(pcl::PointCloud<pcl::PointNormal>::ConstPtr normalcloud,
	                                             const int &search_mum, const float &searchradius)
{
	//创建一个空的kd树对象kdtree，并把它传递给PrincipalCurvatures估计对象。//基于已知的输入数据集，建立kdtree
	pcl::search::KdTree<PointNormal>::Ptr kdtree(new pcl::search::KdTree<PointNormal>);
	pcl::PrincipalCurvaturesEstimation<pcl::PointNormal, pcl::PointNormal, pcl::PrincipalCurvatures> pointCloud_curva;
	pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr cloudcurvatures(new pcl::PointCloud<pcl::PrincipalCurvatures>);
	pointCloud_curva.setInputCloud(normalcloud);
	pointCloud_curva.setInputNormals(normalcloud);
	pointCloud_curva.setSearchMethod(kdtree);
	if (0 != searchradius)
	{
		std::cout << "-------pcl compute_pointcloud_curvatures with a given radius------" << std::endl;
		pointCloud_curva.setRadiusSearch(searchradius);  //以半径搜索附近的点
	}
	else
	{
		std::cout << "-------pcl compute_pointcloud_curvatures with a given number------" << search_mum << std::endl;
		pointCloud_curva.setKSearch(search_mum);         //搜索附近点的数目
	}
	pointCloud_curva.compute(*cloudcurvatures);          //计算点云的主曲率(最大曲率和最小曲率)

	//在微分几何中，在曲面给定点的两个主曲率（principal curvatures）衡量了在给定点一个曲面在这一点的不同方向怎样不同弯曲的程度。
	//在三维欧几里得空间中可微曲面的每一点 p，可选取一个单位法向量。在 p 的一个法平面是包含该法向量以及与曲面相切的惟一一个方向的平面，
	//在曲面上割出一条平面曲线。这条曲线在 p 的不同法平面上一般有不同曲率。在 p的主曲率，记作 k1 与 k2，是这些曲率的最大与最小值。

	//计算每个点的曲率
	vector<float> pointCloud_CVset;
	float curvature = 0.0;
	for (auto icurv : *cloudcurvatures)
	{
		//点的平均曲率
		curvature = (icurv.pc1 + icurv.pc2) / 2;
		//点的高斯曲率
		//curvature = (icurv.pc1 * icurv.pc2;
		pointCloud_CVset.push_back(curvature);
		curvature = 0;
	}
	return pointCloud_CVset;
}

//求点云中每个点的形状指数，计算形状指数需要用到点云的主曲率。首先计算主曲率
std::vector<float> compute_pointcloud_shapeIndex(pcl::PointCloud<pcl::PointNormal>::ConstPtr normalcloud,
	const int &search_mum, const float &searchradius)
{
	//创建一个空的kd树对象kdtree，并把它传递给PrincipalCurvatures估计对象。//基于已知的输入数据集，建立kdtree
	pcl::search::KdTree<PointNormal>::Ptr kdtree(new pcl::search::KdTree<PointNormal>);
	pcl::PrincipalCurvaturesEstimation<pcl::PointNormal, pcl::PointNormal, pcl::PrincipalCurvatures> pointCloud_curva;
	pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr cloudcurvatures(new pcl::PointCloud<pcl::PrincipalCurvatures>);
	pointCloud_curva.setInputCloud(normalcloud);
	pointCloud_curva.setInputNormals(normalcloud);
	pointCloud_curva.setSearchMethod(kdtree);
	if (0 != searchradius)
	{
		std::cout << "-------pcl compute_pointcloud_curvatures with a given radius------" << std::endl;
		pointCloud_curva.setRadiusSearch(searchradius);  //以半径搜索附近的点
	}
	else
	{
		std::cout << "-------pcl compute_pointcloud_curvatures with a given number------" << search_mum << std::endl;
		pointCloud_curva.setKSearch(search_mum);         //搜索附近点的数目
	}
	pointCloud_curva.compute(*cloudcurvatures);          //计算点云的主曲率(最大曲率和最小曲率)

	//在微分几何中，在曲面给定点的两个主曲率（principal curvatures）衡量了在给定点一个曲面在这一点的不同方向怎样不同弯曲的程度。
	//在三维欧几里得空间中可微曲面的每一点 p，可选取一个单位法向量。在 p 的一个法平面是包含该法向量以及与曲面相切的惟一一个方向的平面，
	//在曲面上割出一条平面曲线。这条曲线在 p 的不同法平面上一般有不同曲率。在 p的主曲率，记作 k1 与 k2，是这些曲率的最大与最小值。

	//计算每个点的曲率
	vector<float> cloud_SIset;
	float point_si = 0.0;
	for (auto icurv : *cloudcurvatures)
	{
		//std::cout << "The principle is " << icurv.pc1 << " and " << icurv.pc2 << std::endl;
		//icurv.pc1是主曲率的最大值，icurv.pc2 是主曲率的最小值
		point_si = 0.5 - (1 / 3.1415926)*atan((icurv.pc1 + icurv.pc2) / (icurv.pc1 - icurv.pc2));
		cloud_SIset.push_back(point_si);
		point_si = 0;
	}
	return cloud_SIset;
}

//求点云中每个点的曲率和形状指数
void compute_pointcloud_curvatureAndshapeindex(pcl::PointCloud<pcl::PointNormal>::ConstPtr normalcloud, std::vector<float> &cloud_Curvaset,
	                                           std::vector<float> &cloud_SIset,const int &search_mum, const float &searchradius)
{
	//创建一个空的kd树对象kdtree，并把它传递给PrincipalCurvatures估计对象。//基于已知的输入数据集，建立kdtree
	pcl::search::KdTree<PointNormal>::Ptr kdtree(new pcl::search::KdTree<PointNormal>);
	pcl::PrincipalCurvaturesEstimation<pcl::PointNormal, pcl::PointNormal, pcl::PrincipalCurvatures> pointCloud_curva;
	pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr cloudcurvatures(new pcl::PointCloud<pcl::PrincipalCurvatures>);
	pointCloud_curva.setInputCloud(normalcloud);
	pointCloud_curva.setInputNormals(normalcloud);
	pointCloud_curva.setSearchMethod(kdtree);
	if (0 != searchradius)
	{
		std::cout << "-------pcl compute_pointcloud_curvatures with a given radius------" << std::endl;
		pointCloud_curva.setRadiusSearch(searchradius);  //以半径搜索附近的点
	}
	else
	{
		std::cout << "-------pcl compute_pointcloud_curvatures with a given number------" << search_mum << std::endl;
		pointCloud_curva.setKSearch(search_mum);         //搜索附近点的数目
	}
	pointCloud_curva.compute(*cloudcurvatures);          //计算点云的主曲率(最大曲率和最小曲率)

	//在微分几何中，在曲面给定点的两个主曲率（principal curvatures）衡量了在给定点一个曲面在这一点的不同方向怎样不同弯曲的程度。
	//在三维欧几里得空间中可微曲面的每一点 p，可选取一个单位法向量。在 p 的一个法平面是包含该法向量以及与曲面相切的惟一一个方向的平面，
	//在曲面上割出一条平面曲线。这条曲线在 p 的不同法平面上一般有不同曲率。在 p的主曲率，记作 k1 与 k2，是这些曲率的最大与最小值。

	//计算每个点的曲率和形状指数
	float curvature = 0.0, point_si = 0.0;
	for (auto icurv : *cloudcurvatures)
	{
		//点的平均曲率
		curvature = (icurv.pc1 + icurv.pc2) / 2;
		//点的高斯曲率
		//curvature = (icurv.pc1 * icurv.pc2;
		cloud_Curvaset.push_back(curvature);

		point_si = 0.5 - (1 / 3.1415926)*atan((icurv.pc1 + icurv.pc2) / (icurv.pc1 - icurv.pc2));
		cloud_SIset.push_back(point_si);
		curvature = 0;
		point_si = 0;
	}
}

//求点云中每个点的曲度(点云曲度)
std::vector<float> compute_pointcloud_curvadness(pcl::PointCloud<pcl::PointNormal>::ConstPtr normalcloud,
	const int &search_mum, const float &searchradius)
{
	//创建一个空的kd树对象kdtree，并把它传递给PrincipalCurvatures估计对象。//基于已知的输入数据集，建立kdtree
	pcl::search::KdTree<PointNormal>::Ptr kdtree(new pcl::search::KdTree<PointNormal>);
	pcl::PrincipalCurvaturesEstimation<pcl::PointNormal, pcl::PointNormal, pcl::PrincipalCurvatures> pointCloud_curva;
	pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr cloudcurvatures(new pcl::PointCloud<pcl::PrincipalCurvatures>);
	pointCloud_curva.setInputCloud(normalcloud);
	pointCloud_curva.setInputNormals(normalcloud);
	pointCloud_curva.setSearchMethod(kdtree);
	if (0 != searchradius)
	{
		std::cout << "-------pcl compute_pointcloud_curvatures with a given radius------" << std::endl;
		pointCloud_curva.setRadiusSearch(searchradius);  //以半径搜索附近的点
	}
	else
	{
		std::cout << "-------pcl compute_pointcloud_curvatures with a given number------" << search_mum << std::endl;
		pointCloud_curva.setKSearch(search_mum);         //搜索附近点的数目
	}
	pointCloud_curva.compute(*cloudcurvatures);          //计算点云的主曲率(最大曲率和最小曲率)

	//计算每个点的曲度
	vector<float> curvadness_set;
	float curvadness = 0.0;
	for (auto icurv : *cloudcurvatures)
	{
		//点的曲度
		curvadness = sqrt((icurv.pc1*icurv.pc1 + icurv.pc2*icurv.pc2) / 2);
		curvadness_set.push_back(curvadness);
		curvadness = 0;
	}
	return curvadness_set;
}

//传入PointNormal云，利用待处理点周围 search_number 个搜索点求点的曲率,并将前 scale (0,1)范围的点做标记
pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr compute_pointscurvature_colorpoints
(pcl::PointCloud<pcl::PointNormal>::ConstPtr normalcloud, const int &search_number,
	const float &radius, const float &scale)
{
	//利用PointXYZ点云和normal点云,获取点云的曲率集
	vector<float> cloud_Curaset;
	cloud_Curaset = compute_pointcloud_curvatures(normalcloud, search_number, radius);
	std::vector<float> cloud_cura_maxAndmin;
	cloud_cura_maxAndmin = compute_maxAndmin2vector(cloud_Curaset);
	float criteria_clou1_cura_1= 0, criteria_clou1_cura_2 = 0;

	criteria_clou1_cura_1 = cloud_cura_maxAndmin[1] + (1 - 0.4)*(cloud_cura_maxAndmin[0] - cloud_cura_maxAndmin[1]);
	criteria_clou1_cura_2 = cloud_cura_maxAndmin[1] + (1 - scale)*(cloud_cura_maxAndmin[0] - cloud_cura_maxAndmin[1]);

	//将normal云转换为rgbnormal云，然后可视化，曲率较大的点显示为红色,其他的显示为蓝色
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr normalrgbcloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	transform_normal2rgbnormal(normalcloud, normalrgbcloud);

	for (int i = 0; i<normalcloud->size(); i++)
	{
		if (cloud_Curaset[i]>criteria_clou1_cura_2)
		{
			normalrgbcloud->points[i].r = 255;
			normalrgbcloud->points[i].g = 0;
			normalrgbcloud->points[i].b = 0;
		}
		else
		{
			normalrgbcloud->points[i].r = 0;
			normalrgbcloud->points[i].g = 0;
			normalrgbcloud->points[i].b = 255;
		}
	}
	return normalrgbcloud;
}

//输入一个向量,求出最大值和最小值。然后把最值放入一个二维向量，返回
std::vector<float> compute_maxAndmin2vector(std::vector<float> &Vec)
{
	std::vector<float> max_min;
	float max_Vec = -10000, min_Vec = 10000;
	for (int icura = 0; icura<Vec.size(); icura++)
	{
		if (Vec[icura]>max_Vec)
		{
			max_Vec = Vec[icura];
		}

		if (Vec[icura] < min_Vec)
		{
			min_Vec = Vec[icura];
		}
	}
	max_min.push_back(max_Vec);
	max_min.push_back(min_Vec);
	return max_min;
}


//传入PointNormal云,利用待处理点周围半径search_radius内的搜索点求点的曲率,并根据曲率范围的做颜色标记
pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr compute_pointscurvature_markpoints
(pcl::PointCloud<pcl::PointNormal>::ConstPtr normalcloud, const int &searchnum,
	const float &search_radius, const float &curvature_max, const float &curvature_min)
{
	std::cout << "compute_pointscurvature_markpoints(......)" << std::endl;

	//利用PointXYZ点云和点云的normal,获取点云的曲率集
	vector<float> cloud_CVset;
	cloud_CVset = compute_pointcloud_curvatures(normalcloud, searchnum, search_radius);

	//将normal云转换为rgbnormal云，然后可视化
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr normalrgbcloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	transform_normal2rgbnormal(normalcloud, normalrgbcloud);

	//为点云着色,曲率较大的为红色,曲率较小的为蓝色。中间的为绿色
	for (int i = 0; i<normalcloud->size(); i++)
	{
		std::cout << i << " point's curature: " << cloud_CVset[i] << std::endl;
		if (cloud_CVset[i]>curvature_max)
		{
			normalrgbcloud->points[i].r = 255;
			normalrgbcloud->points[i].g = 0;
			normalrgbcloud->points[i].b = 0;
		}
		else if (cloud_CVset[i] <= curvature_min)
		{
			normalrgbcloud->points[i].r = 0;
			normalrgbcloud->points[i].g = 0;
			normalrgbcloud->points[i].b = 255;
		}
		else
		{
			normalrgbcloud->points[i].r = 0;
			normalrgbcloud->points[i].g = 255;
			normalrgbcloud->points[i].b = 0;
		}
	}
	return normalrgbcloud;
}









// The idea refer to the Coordinate Transformation method and its modition
//References：1、用改进的CT方法估算离散点云的法矢和曲率   2、利用点云数据的法矢及曲率估算 3、曲面重构中散乱点云数据曲率估算算法的研究 
//4、基于 4D Shepard 曲面的点云曲率估算


//计算整个点云的曲率
std::vector<float> compute_curature2cloud(pcl::PointCloud<pcl::PointNormal>::ConstPtr normcloud, const double &searchradius, const int &search_num)
{
	std::cout << "compute_curature2cloud by fitting surface" << std::endl;
	//创建KdTreeFLANN对象，并把创建的点云设置为输入
	pcl::KdTreeFLANN<pcl::PointNormal> kdtreeCura;
	kdtreeCura.setInputCloud(normcloud);

	//------------------------------计算点的曲率所需要的准备工作------------------------------------
	Eigen::MatrixXf MC = Eigen::MatrixXf::Zero(1000, 5);           //坐标变换后，计算曲率时的矩阵  MC*x= b
	Eigen::VectorXf rightHand = Eigen::VectorXf::Zero(1000);       //坐标变换后，计算曲率时的右端项

	Eigen::Vector3f aixs2point = Eigen::Vector3f::Zero(3);        //坐标变换前点的坐标
	Eigen::Vector3f update_aixs2point = Eigen::Vector3f::Zero(3); //坐标变换后点的坐标

	std::vector<int> pointIdxNKNSearch;                           //存储查询点近邻索引
	std::vector<float> pointNKNSquaredDistance;                   //存储近邻点对应距离平方

	float K_gauss = 0, K_mean = 0;                                //高斯曲率和平均曲率
	Eigen::VectorXf curature_para = Eigen::VectorXf::Ones(5);     //存储计算曲率的5个参数

	//旋转矩阵变量,并用单位矩阵初始化
	Eigen::Matrix3f rotaMatrix = Eigen::Matrix3f::Identity(3, 3);

	std::vector<float> mean_curature_set;
	//计算点的曲率,利用坐标变换，将点转换到局部坐标系下，然后在局域内构造抛物面
	for (int ipoint = 0; ipoint < normcloud->size(); ipoint++)
	{
		//std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ ipoint: " << ipoint << std::endl;
		Eigen::Vector3f base_point_normal(normcloud->points[ipoint].normal[0], normcloud->points[ipoint].normal[1], normcloud->points[ipoint].normal[2]);
		//判断法向量是否归一化，标准如果|norm-1|>0.0001，就认为没有归一化。进行归一化
		if(abs(base_point_normal.norm()-1)>1e-4)
		{
			base_point_normal(0) = base_point_normal(0) / base_point_normal.norm();
			base_point_normal(1) = base_point_normal(1) / base_point_normal.norm();
			base_point_normal(2) = base_point_normal(2) / base_point_normal.norm();
		}

		Eigen::Vector3f base_point_aix(normcloud->points[ipoint].x, normcloud->points[ipoint].y, normcloud->points[ipoint].z);

		Eigen::Vector3f aixZ_vec(0, 0, 1), new_basepoint_aix(0,0, 0);
		rotaMatrix = comppute_rotaMatrixByrotaVector(base_point_normal, aixZ_vec);//z轴沿z轴和法向的夹角a顺时针旋转
		new_basepoint_aix = rotaMatrix*base_point_aix;
		
		//new_basepoint_aix(0) = 0;
		//new_basepoint_aix(1) = 0;
		//new_basepoint_aix(2) = new_basepoint_aix.norm();
		//std::cout << "new_basepoint_aix:" << std::endl;
		//std::cout << new_basepoint_aix << std::endl;

		int search_result = 0;
		if (0 != searchradius)   //如果搜索半径不等于0，就按照半径搜索，否则按照点数搜索
		{
			search_result = kdtreeCura.radiusSearch(normcloud->points[ipoint], searchradius, pointIdxNKNSearch, pointNKNSquaredDistance);
		}
		else
		{
			search_result = kdtreeCura.nearestKSearch(normcloud->points[ipoint], search_num, pointIdxNKNSearch, pointNKNSquaredDistance);
		}

		if (0 == search_result)
		{
			break;
		}

		int index = 0;
		//kdtree搜索附近的点
		if (search_result > 0 && pointIdxNKNSearch.size() > 6 && (0 != searchradius))
		{
			//std::cout << "Search by radius and the number of haveing searched points:" << pointIdxNKNSearch.size() << std::endl;
			for (int j = 0; j < pointIdxNKNSearch.size(); j++)
			{
				if (pointIdxNKNSearch[j] != ipoint)
				{
					//提取待转转换临近点的坐标
					aixs2point(0) = normcloud->points[pointIdxNKNSearch[j]].x;
					aixs2point(1) = normcloud->points[pointIdxNKNSearch[j]].y;
					aixs2point(2) = normcloud->points[pointIdxNKNSearch[j]].z;

					//点的左边转换(旋转和平移，先旋转后平移)
					update_aixs2point = aixs_transform(rotaMatrix, new_basepoint_aix, aixs2point);

					MC(index, 0) = update_aixs2point(0)*update_aixs2point(0);
					MC(index, 1) = update_aixs2point(0)*update_aixs2point(1);
					MC(index, 2) = update_aixs2point(1)*update_aixs2point(1);
					MC(index, 3) = update_aixs2point(0);
					MC(index, 4) = update_aixs2point(1);
					rightHand(index) = update_aixs2point(2);
					index++;
				}
			}
		}
		else //if( kdtreeCura.nearestKSearch(normcloud->points[ipoint], search_num, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
		{
			//std::cout << "-----------------Search by search_num--------------------" << std::endl;
			for (int j = 0; j < pointIdxNKNSearch.size(); j++)
			{
				if (pointIdxNKNSearch[j] != ipoint)
				{
					//提取待转转换临近点的坐标
					aixs2point(0) = normcloud->points[pointIdxNKNSearch[j]].x;
					aixs2point(1) = normcloud->points[pointIdxNKNSearch[j]].y;
					aixs2point(2) = normcloud->points[pointIdxNKNSearch[j]].z;

					//点的左边转换(旋转和平移，先旋转后平移)
					update_aixs2point = aixs_transform(rotaMatrix, new_basepoint_aix, aixs2point);

					MC(index, 0) = update_aixs2point(0)*update_aixs2point(0);
					MC(index, 1) = update_aixs2point(0)*update_aixs2point(1);
					MC(index, 2) = update_aixs2point(1)*update_aixs2point(1);
					MC(index, 3) = update_aixs2point(0);
					MC(index, 4) = update_aixs2point(1);
					rightHand(index) = update_aixs2point(2);
					index++;
				}
			}
		}

		//求解计算曲率的5个参数
		curature_para = ((MC.transpose())*MC).colPivHouseholderQr().solve(-(MC.transpose())*rightHand);//这里右端项要有负号
		//计算高斯曲率和平均曲率
		compute_curature2point(curature_para, K_gauss, K_mean);
		mean_curature_set.push_back(K_mean);

	}
	return mean_curature_set;
}

//计算高斯曲率和平均曲率
void compute_curature2point(Eigen::VectorXf &cura_paravec, float &gauss_cura, float &mean_cura)
{	
	//cura_paravec = ( a，b，c，d，e) 
	float E=1, F=2, G=3, L=4, M=5, N=6;
	E = 1 + cura_paravec[3] * cura_paravec[3]; //1+d*d
	F = cura_paravec[3] * cura_paravec[4];     //d*e
	G = 1 + cura_paravec[4] * cura_paravec[4];
	L = (2 * cura_paravec[0]) / sqrt(1 + cura_paravec[3] * cura_paravec[3] + cura_paravec[4] * cura_paravec[4]);
	M = cura_paravec[1] / sqrt(1 + cura_paravec[3] * cura_paravec[3] + cura_paravec[4] * cura_paravec[4]);
	N = (2 * cura_paravec[2]) / sqrt(1 + cura_paravec[3] * cura_paravec[3] + cura_paravec[4] * cura_paravec[4]);

//	gauss_cura = (L*N - M*M) / (E*G - F*F);
//	mean_cura = 0.5*((E*N - 2 * F*M + G*L) / (E*G - F*F));//为什么会出现负的呢？
	gauss_cura = abs((L*N - M*M) / (E*G - F*F));
	mean_cura = abs(0.5*((E*N - 2 * F*M + G*L) / (E*G - F*F)));
}


//坐标变换(旋转和平移)··················先旋转后平移!·····················································
Eigen::Vector3f aixs_transform(Eigen::Matrix3f &Rota, Eigen::Vector3f &afterRota_basepoint, Eigen::Vector3f &origin_aix2point)
{
	
	Eigen::Vector3f new_aix(0, 0, 0);

	//进行旋转变换
	new_aix = Rota*origin_aix2point;

	//进行平移(相对于旋转后的基点进行平移)
	if ((afterRota_basepoint.norm())>(1e-6))
	{
		new_aix[0] = new_aix[0] - afterRota_basepoint[0];
		new_aix[1] = new_aix[1] - afterRota_basepoint[1];
		new_aix[2] = new_aix[2] - afterRota_basepoint[2];
	}
	return new_aix;	
}

//利用两个向量计算旋转矩阵 u向v旋转
Eigen::Matrix3f comppute_rotaMatrixByrotaVector(Eigen::Vector3f &u, Eigen::Vector3f &v)
{
	/*
	//u,v为3行1列的列向量
		std::cout << "u:  ";
		std::cout << u(0) << "  " << u(1) << "  " << u(2) << std::endl;
		std::cout << "v:  ";
		std::cout << v(0) << "  " << v(1) << "  " << v(2) << std::endl;
	*/

	Eigen::Matrix3f RotaM = Eigen::Matrix3f::Zero(3,3);
	
	//叉乘求旋转轴,并把旋转轴归一化
	Eigen::Vector3f crossVec = u.cross(v);
	crossVec(0) = crossVec(0) / crossVec.norm();
	crossVec(1) = crossVec(1) / crossVec.norm();
	crossVec(2) = crossVec(2) / crossVec.norm();
	//std::cout << "crossVec:  ";
	//std::cout << crossVec(0) << "  " << crossVec(1) << "  " << crossVec(2) << std::endl;

	//夹角的余弦值
	float costheta = (u.dot(v)) / sqrt(u.norm())*sqrt(v.norm());

	if (abs(1- costheta)<1e-5)
	{
		RotaM(0, 0) = 1;
		RotaM(0, 1) = 0;
		RotaM(0, 2) = 0;

		RotaM(1, 0) = 0;
		RotaM(1, 1) = 1;
		RotaM(1, 2) = 0;

		RotaM(2, 0) = 0;
		RotaM(2, 1) = 0;
		RotaM(2, 2) = 1;
	}
	else if (abs(-1 - costheta)<1e-5)
	{
		RotaM(0, 0) = 1;
		RotaM(0, 1) = 0;
		RotaM(0, 2) = 0;

		RotaM(1, 0) = 0;
		RotaM(1, 1) = -1;
		RotaM(1, 2) = 0;

		RotaM(2, 0) = 0;
		RotaM(2, 1) = 0;
		RotaM(2, 2) = -1;
	}
	else
	{		
		//rotation vector to  rotation matrix
		float theta = acos(costheta);
		//std::cout << "theta: " << theta << std::endl;
		Eigen::AngleAxisf rotationVector(theta, crossVec);
		RotaM = rotationVector.toRotationMatrix();
	}
	/*
	for (int iRM =0;iRM<3;iRM++)
	{
		for (int jRM = 0; jRM < 3; jRM++)
		{
			if (RotaM(iRM, jRM)<0 &&abs(RotaM(iRM, jRM))<(7*1e-8))
			{
				RotaM(iRM, jRM) = 0;
			}
		}
	}
	std::cout << "RotationMatrix:" << std::endl;
	std::cout << RotaM(0, 0) << "  " << RotaM(0, 1) << "  " << RotaM(0, 2) << std::endl;
	std::cout << RotaM(1, 0) << "  " << RotaM(1, 1) << "  " << RotaM(1, 2) << std::endl;
	std::cout << RotaM(2, 0) << "  " << RotaM(2, 1) << "  " << RotaM(2, 2) << std::endl;
	*/
	return RotaM;
}



/*
//绕给定的过原点的方向向量的旋转，求得旋转变换矩阵(xyz坐标轴的旋转)
Eigen::Matrix3f compute_rotamatrix(Eigen::Vector3f &normal_temp, Eigen::Vector3f &origin_axi)
{
	float squre_normal_temp = normal_temp[0] * normal_temp[0] + normal_temp[1] * normal_temp[1] + normal_temp[2] * normal_temp[2];
	float squre_origin_axi = origin_axi[0] * origin_axi[0] + origin_axi[1] * origin_axi[1] + origin_axi[2] * origin_axi[2];
	float innerprodut = normal_temp.dot(origin_axi); // normal_temp[0] * origin_axi[0] + normal_temp[1] * origin_axi[1] + normal_temp[2] * origin_axi[2];
	float costheta = innerprodut / (sqrt(squre_normal_temp)*sqrt(squre_origin_axi)); //两个向量的夹角

	Eigen::Matrix3f Diagional_cos, Diagional_sin;
	Diagional_cos(0, 0) = Diagional_cos(1, 1) = Diagional_cos(2, 2) = costheta;
	Diagional_cos(0, 1) = Diagional_cos(0, 2) = Diagional_cos(1, 0) = Diagional_cos(1, 2) = Diagional_cos(2, 0) = Diagional_cos(2, 1) = 0;

	Diagional_sin(0, 0) = Diagional_sin(1, 1) = Diagional_sin(2, 2) = sqrt(1 - costheta*costheta);
	Diagional_sin(0, 1) = Diagional_sin(0, 2) = Diagional_sin(1, 0) = Diagional_sin(1, 2) = Diagional_sin(2, 0) = Diagional_sin(2, 1) = 0;

	Eigen::Matrix3f Identity;
	Identity(0, 1) = Identity(0, 2) = Identity(1, 0) = Identity(1, 2) = Identity(2, 0) = Identity(2, 1) = 0;
	Identity(0, 0) = Identity(1, 1) = Identity(2, 2) = 1;

	Eigen::Matrix3f M1, M2;
	M1(0, 0) = normal_temp[0] * normal_temp[0];
	M1(0, 1) = normal_temp[0] * normal_temp[1];
	M1(0, 2) = normal_temp[0] * normal_temp[2];

	M1(1, 0) = normal_temp[1] * normal_temp[0];
	M1(1, 1) = normal_temp[1] * normal_temp[1];
	M1(1, 2) = normal_temp[1] * normal_temp[2];

	M1(2, 0) = normal_temp[2] * normal_temp[0];
	M1(2, 1) = normal_temp[2] * normal_temp[1];
	M1(2, 2) = normal_temp[2] * normal_temp[2];

	M2(0, 0) = 0;
	M2(0, 1) = -normal_temp[2];
	M2(0, 1) = normal_temp[1];

	M2(0, 0) = normal_temp[2];
	M2(0, 1) = 0;
	M2(0, 1) = -normal_temp[0];

	M2(0, 0) = -normal_temp[1];
	M2(0, 1) = normal_temp[0];
	M2(0, 1) = 0;

	return (M1 + Diagional_cos*(Identity - M1) + Diagional_sin*M2);
}

//利用给定的一个过原点的向量求xoy到以该向量正方向为‘Z’轴的旋转矩阵
Eigen::Matrix3f compute_rotamatrix(Eigen::Vector3f &origin_pointVec)
{
	Eigen::Vector3f xoy_normal(0, 0, 1), xoz_normal(0, 1, 0), yoz_normal(1, 0, 0);
	Eigen::Vector3f vec2xoy, vec2xoz, vec2yoz;//用于记录向量到各坐标平面的投影
	vec2xoy = compute_projectVector2aixplane(xoy_normal, origin_pointVec);
	vec2xoz = compute_projectVector2aixplane(xoz_normal, origin_pointVec);
	vec2yoz = compute_projectVector2aixplane(yoz_normal, origin_pointVec);
	Eigen::Matrix3f R_xoy, R_xoz, R_yoz;
	R_xoy = compute_rotaMatrix2aix(xoy_normal, vec2xoy);
	R_xoz = compute_rotaMatrix2aix(xoz_normal, vec2xoz);
	R_yoz = compute_rotaMatrix2aix(yoz_normal, vec2yoz);
	return R_yoz*R_xoz*R_xoy;
}

//计算空间中一个过原点的向量到坐标平面的投影向量
Eigen::Vector3f compute_projectVector2aixplane(Eigen::Vector3f &aix_normal, Eigen::Vector3f &givenVec)
{
	Eigen::Vector3f projectVec;
	//投影到yoz平面
	if (1 == aix_normal(0))
	{
		projectVec(0) = 0;
		projectVec(1) = givenVec(1);
		projectVec(2) = givenVec(2);
	}
	//投影到xoz平面
	else if (1 == aix_normal(1))
	{
		projectVec(0) = givenVec(0);
		projectVec(1) = 0;
		projectVec(2) = givenVec(2);
	}
	//投影到xoy平面
	else //(1 == aix_normal(2))
	{
		projectVec(0) = givenVec(0);
		projectVec(1) = givenVec(1);
		projectVec(2) = 0;
	}
	return projectVec;
}

//计算xoy平面绕z坐标轴矩阵，或者xoz平面绕y坐标轴矩阵，或者xoy平面绕z坐标轴矩阵
Eigen::Matrix3f compute_rotaMatrix2aix(Eigen::Vector3f &aix_normal, Eigen::Vector3f &planeVec)
{
	Eigen::Matrix3f Rota;
	float costheta = 0, sintheta = 0;
	float innerProduct = (aix_normal.transpose()).dot(planeVec);
	float norm_product = sqrt(aix_normal[0] * aix_normal[0] + aix_normal[1] * aix_normal[1] + aix_normal[2] * aix_normal[2])*sqrt(planeVec[0] * planeVec[0] + planeVec[1] * planeVec[1] + planeVec[2] * planeVec[2]);
	costheta = innerProduct / norm_product;
	sintheta = sqrt(1 - costheta*costheta);
	//yoz平面中的向量绕x轴旋转
	if (1 == aix_normal(0))
	{
		Rota(0, 0) = 1;
		Rota(0, 1) = 0;
		Rota(0, 2) = 0;

		Rota(1, 0) = 0;
		Rota(1, 1) = costheta;
		Rota(1, 2) = -sintheta;

		Rota(2, 0) = 0;
		Rota(2, 1) = sintheta;
		Rota(2, 2) = costheta;
	}
	//xoz平面中的向量绕y轴旋转
	else if (1 == aix_normal(1))
	{
		Rota(0, 0) = costheta;
		Rota(0, 1) = 0;
		Rota(0, 2) = sintheta;

		Rota(1, 0) = 0;
		Rota(1, 1) = 1;
		Rota(1, 2) = 0;

		Rota(2, 0) = -sintheta;
		Rota(2, 1) = 0;
		Rota(2, 2) = costheta;
	}
	//xoy平面中的向量绕z轴旋转
	else //(1 == aix_normal(2))
	{
		Rota(0, 0) = costheta;
		Rota(0, 1) = -sintheta;
		Rota(0, 2) = 0;

		Rota(1, 0) = sintheta;
		Rota(1, 1) = costheta;
		Rota(1, 2) = 0;

		Rota(2, 0) = 0;
		Rota(2, 1) = 0;
		Rota(2, 2) = 1;
	}
	return Rota;
}
*/


