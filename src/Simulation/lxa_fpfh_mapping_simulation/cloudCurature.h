#pragma once
#include"common.h"

//--------------------------------------------------------------------------------------------------------------------------------------------------
//                          以下pcld点云库的代码实现点云曲率计算
//--------------------------------------------------------------------------------------------------------------------------------------------------

//求点云中每个点的曲率(点云曲率)根据点数
std::vector<float> compute_pointcloud_curvatures(pcl::PointCloud<pcl::PointNormal>::ConstPtr normalcloud,
	const int &search_mum, const float &searchradius);

//求点云中每个点的形状指数，计算形状指数需要用到点云的主曲率。首先计算主曲率
std::vector<float> compute_pointcloud_shapeIndex(pcl::PointCloud<pcl::PointNormal>::ConstPtr normalcloud,
	const int &search_mum, const float &searchradius);


//求点云中每个点的曲率和形状指数
void compute_pointcloud_curvatureAndshapeindex(pcl::PointCloud<pcl::PointNormal>::ConstPtr normalcloud, std::vector<float> &cloud_Curvaset,
	std::vector<float> &cloud_SIset, const int &search_mum, const float &searchradius);

//求点云中每个点的曲度(点云曲度)
std::vector<float> compute_pointcloud_curvadness(pcl::PointCloud<pcl::PointNormal>::ConstPtr normalcloud,
	const int &search_mum, const float &searchradius);


//输入一个向量,求出最大值和最小值。然后把最值放入一个二维向量，返回
std::vector<float> compute_maxAndmin2vector(std::vector<float> &Vec);

//传入normal云，利用待处理点周围 search_number 个搜索点求点的曲率,并将前 scale (0,1)范围的点做标记
pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr compute_pointscurvature_colorpoints
(pcl::PointCloud<pcl::PointNormal>::ConstPtr normalcloud, const int &search_number,
	const float &radius, const float &scale);

//传入normal云，利用待处理点周围半径search_radius内的搜索点求点的曲率,并根据曲率范围的做颜色标记
pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr compute_pointscurvature_markpoints
(pcl::PointCloud<pcl::PointNormal>::ConstPtr normalcloud, const int &searchnum,
	const float &search_radius, const float &curvature_max, const float &curvature_min);



//--------------------------------------------------------------------------------------------------------------------------------------------------
//                          以下为自己编写的代码实现点云曲率计算
//--------------------------------------------------------------------------------------------------------------------------------------------------


//计算整个点云的曲率
std::vector<float> compute_curature2cloud(pcl::PointCloud<pcl::PointNormal>::ConstPtr normcloud, const double &searchradius, const int &search_num);

//坐标变换(旋转和平移)··················先旋转后平移!·····················································
Eigen::Vector3f aixs_transform(Eigen::Matrix3f &Rota, Eigen::Vector3f &base_point, Eigen::Vector3f &origin_aix2point);

//利用两个向量计算旋转矩阵
Eigen::Matrix3f comppute_rotaMatrixByrotaVector(Eigen::Vector3f &u, Eigen::Vector3f &v);

//计算高斯曲率和平均曲率
void compute_curature2point(Eigen::VectorXf &cura_paravec, float &gauss_cura, float &mean_cura);


//std::vector<float> solve_Matrix_system(Eigen::MatrixXf A, Eigen::VectorXf b);

/*
Eigen::Matrix3f compute_rotamatrix(Eigen::Vector3f &normal_temp, Eigen::Vector3f &origin_axi);

//利用给定的一个过原点的向量求xoy到以该向量正方向为‘Z’轴的旋转矩阵
Eigen::Matrix3f compute_rotamatrix(Eigen::Vector3f &origin_pointVec);

//计算空间中一个过原点的向量到坐标平面的投影向量
Eigen::Vector3f compute_projectVector2aixplane(Eigen::Vector3f &aix_normal, Eigen::Vector3f &givenVec);

//计算xoy平面绕z坐标轴矩阵，或者xoz平面绕y坐标轴矩阵，或者xoy平面绕z坐标轴矩阵
Eigen::Matrix3f compute_rotaMatrix2aix(Eigen::Vector3f &aix_normal, Eigen::Vector3f &planeVec);
*/



