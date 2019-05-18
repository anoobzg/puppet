#pragma once
#include "common.h"
#include "basetools.h"

//------------------------------------------------------------------------------------------
void load_pointCloud_show(std::string plypath);                        

void load_pointCloud_showPoint(std::string filepath, bool ifprintPoint);


//--------------------------------------根据路径读入点云--------------------------------------
void load_pointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud, std::string filepath);

void load_pointcloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud, std::string filepath);

void load_pointcloud(pcl::PointCloud<pcl::PointNormal>::Ptr pointcloud, std::string filepath);

//------------------------------3D 可视化点云----------------------------
void viewcloud_3D(pcl::PointCloud<pcl::PointXYZ>::ConstPtr pointcloud);

void viewcloud_3D(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pointcloud);

void viewcloud_3D(pcl::PointCloud<pcl::PointNormal>::ConstPtr pointcloud);

void viewcloud_3D(pcl::PointCloud<pcl::PointXYZRGBNormal>::ConstPtr pointcloud);

void view_two_cloud_3D(pcl::PointCloud<pcl::PointNormal>::ConstPtr pointcloud1, 
	                   pcl::PointCloud<pcl::PointNormal>::ConstPtr pointcloud2);

//----------------------------------------------点云的变化--------------------------------------------------------------
//将PointNormal点云转换为PointXYZRGBNormal点云
void transform_normal2rgbnormal(pcl::PointCloud<pcl::PointNormal>::ConstPtr normalCloud, 
	                            pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr rgbnormalCloud);

//提取PointNormal云中的点XYZ存储为PointXYZ点云
pcl::PointCloud<pcl::PointXYZ>::Ptr extract_XYZcloud_PointNormal(pcl::PointCloud<pcl::PointNormal>::ConstPtr normcloud);

//提取PointXYZRGBNormal云中的点XYZ存储为PointXYZ点云
pcl::PointCloud<pcl::PointXYZ>::Ptr extract_XYZcloud_PointNormal(pcl::PointCloud<pcl::PointXYZRGBNormal>::ConstPtr rgbnormcloud);

//将PointNormal点云中的normal和xyz提取,存储为normal点云
pcl::PointCloud<pcl::Normal>::Ptr extract_Normal_PointNormal(pcl::PointCloud<pcl::PointNormal>::ConstPtr normcloud);

//将PointXYZRGBNormal点云中的normal和xyz提取,存储为normal点云
pcl::PointCloud<pcl::Normal>::Ptr extract_Normal_PointNormal(pcl::PointCloud<pcl::PointXYZRGBNormal>::ConstPtr rgbnormcloud);


//显示mormal点的法向量(按x,y,z查看)
void show_pointNormal(pcl::PointNormal normalpoint);

//显示rgbmormal点的法向量(按x,y,z查看)
void show_rgbpointNormal(pcl::PointXYZRGBNormal rgbnormalpoint);

//计算xyz点云的法线,并将结果赋值到normalcloud点云
void compute_normal2cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const float search_radius, 
	                      pcl::PointCloud<pcl::PointNormal>::Ptr normalcloud);

//旋转点云
void rota_cloud(pcl::PointCloud<pcl::PointNormal>::Ptr normalcloud_in, 
	            pcl::PointCloud<pcl::PointNormal>::Ptr normalcloud_out, float rota_angle);

//稀疏话点云(从点云中删除部分点)
void saprse_cloud(pcl::PointCloud<pcl::PointNormal>::ConstPtr normalcloud_orginal,
	pcl::PointCloud<pcl::PointNormal>::Ptr normalcloud_new, const int dismiss_num);
