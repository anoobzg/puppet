#include "pointCloudbase.h"
//根据路径读入ply数据(以PointXYZ读取)，然后显示点云
void load_pointCloud_show(std::string plypath)
{
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>); // 创建点云（指针） 
	//if (pcl::io::loadPLYFile<pcl::PointXYZ>(plypath, *cloud) == -1)
	//{
	//	PCL_ERROR("Couldn't read ply file \n"); //文件不存在时，返回错误，终止程序。  
	//	return;
	//}
	//// 3d可视化点云
	//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Cloud Viewer"));
	//viewer->addPointCloud<pcl::PointXYZ>(cloud, "cloud");
	//while (!viewer->wasStopped())
	//{
	//	viewer->spinOnce(100);
	//	boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	//}
}

//根据路径读入ply数据，然后转换为pcd类型，最后显示点云
void load_pointCloud_showPoint(std::string filepath, bool ifprintPoint)
{
	//pcl::PLYReader reader;
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>); //创建xyz点云(指针)
	//int read_result = reader.read<pcl::PointXYZ>(filepath, *cloud);                //读入ply点云文件存储到cloud里面
	//if (-1 == read_result)
	//{
	//	PCL_ERROR("Couldn't read file test_pcd.pcd \n"); //文件不存在时，返回错误，终止程序。  
	//	return;
	//}
	////pcl::io::savePCDFile(filepath, *cloud);                                      //保存点云到 pointcloud.pcd 文件中
	//
	////显示所有的点(x,y,z) 
	//if (true == ifprintPoint)
	//{
	//	std::cout << "Loaded " << cloud->width * cloud->height << " data points from test_file.pcd with the following fields: " << std::endl;
	//	for (size_t i = 0; i < cloud->size(); ++i)
	//	{
	//		std::cout << "    " << cloud->points[i].x << " " << cloud->points[i].y << " " << cloud->points[i].z << std::endl;
	//	}
	//}
	//
	//// 3D可视化点云
	//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Cloud Viewer"));
	//viewer->addPointCloud<pcl::PointXYZ>(cloud, "cloud");
	//while (!viewer->wasStopped())
	//{
	//	viewer->spinOnce(100);
	//	boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	//}
}

//根据路径读入ply数据(以PointXYZ读取)
void load_pointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud, std::string filepath)
{
	//if (pcl::io::loadPLYFile<pcl::PointXYZ>(filepath, *pointcloud) == -1)
	//{
	//	PCL_ERROR("Couldn't read ply file \n"); //文件不存在时，返回错误，终止程序。
	//	return;
	//}
	return;
}

//根据路径读入ply数据(以PointXYZRGB读取)
void load_pointcloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud, std::string filepath)
{
	//if (pcl::io::loadPLYFile<pcl::PointXYZRGB>(filepath, *pointcloud) == -1)
	//{
	//	PCL_ERROR("Couldn't read ply file \n"); //文件不存在时，返回错误，终止程序。 
	//	return;
	//}
	return;
}

//根据路径读入ply数据(以PointNormal读取)
void load_pointcloud(pcl::PointCloud<pcl::PointNormal>::Ptr pointcloud, std::string filepath)
{
	//if (pcl::io::loadPLYFile<pcl::PointNormal>(filepath, *pointcloud) == -1)
	//{
	//	PCL_ERROR("Couldn't read ply file \n"); //文件不存在时，返回错误，终止程序。 
	//	return;
	//}
	return;
}

// 3d可视化点云(xyz云)
void viewcloud_3D(pcl::PointCloud<pcl::PointXYZ>::ConstPtr pointcloud)
{
	//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Cloud Viewer"));
	//viewer->setBackgroundColor(0, 0, 0);  //设置背景颜色为黑色
	//viewer->addCoordinateSystem(1.0); //建立空间直角坐标系
	//viewer->addPointCloud<pcl::PointXYZ>(pointcloud, "cloud");
	//
	//// 等待直到可视化窗口关闭
	//while (!viewer->wasStopped())
	//{
	//	viewer->spinOnce(100);
	//	boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	//}
	//一个面显示
	//pcl::visualization::CloudViewer viewer("ply Viewer");
	//viewer.showCloud(cloud);
	//system("pause");
}

// 3d可视化点云(xyzrgb云)
void viewcloud_3D(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pointcloud)
{
	//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Cloud Viewer"));
	//viewer->setBackgroundColor(0, 0, 0);  //设置背景颜色为黑色
	//viewer->addPointCloud<pcl::PointXYZRGB>(pointcloud, "rgbcloud");
	//
	//// 等待直到可视化窗口关闭
	//while (!viewer->wasStopped())
	//{
	//	viewer->spinOnce(100);
	//	boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	//}
	//一个面显示
	//pcl::visualization::CloudViewer viewer("ply Viewer");
	//viewer.showCloud(cloud);
	//system("pause");
}

// 3d可视化点云(normal云)
void viewcloud_3D(pcl::PointCloud<pcl::PointNormal>::ConstPtr pointcloud)
{
	//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Cloud Viewer"));
	//viewer->setBackgroundColor(0, 0, 0);  //设置背景颜色为黑色
	//viewer->addPointCloud<pcl::PointNormal>(pointcloud, "normalcloud");
	//
	//// 等待直到可视化窗口关闭
	//while (!viewer->wasStopped())
	//{
	//	viewer->spinOnce(100);
	//	boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	//}
	//一个面显示
	//pcl::visualization::CloudViewer viewer("ply Viewer");
	//viewer.showCloud(cloud);
	//system("pause");
}

// 3d可视化点云(xyzrgbnormal云)
void viewcloud_3D(pcl::PointCloud<pcl::PointXYZRGBNormal>::ConstPtr pointcloud)
{
	//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Cloud Viewer"));
	//viewer->setBackgroundColor(0, 0, 0);  //设置背景颜色为黑色
	//viewer->addPointCloud<pcl::PointXYZRGBNormal>(pointcloud, "normalrgbcloud");
	//// 等待直到可视化窗口关闭
	//while (!viewer->wasStopped())
	//{
	//	viewer->spinOnce(100);
	//	boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	//}
}

void view_two_cloud_3D(pcl::PointCloud<pcl::PointNormal>::ConstPtr pointcloud1,
	                   pcl::PointCloud<pcl::PointNormal>::ConstPtr pointcloud2)
{
	//boost::shared_ptr<pcl::visualization::PCLVisualizer> view(new pcl::visualization::PCLVisualizer("viewer"));  //定义窗口共享指针
	//int v1; //定义两个窗口v1，v2，窗口v1用来显示初始位置，v2用以显示配准过程
	//int v2;
	//view->createViewPort(0.0, 0.0, 0.5, 1.0, v1);  //四个窗口参数分别对应x_min,y_min,x_max.y_max.
	//view->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	//
	//view->addPointCloud<pcl::PointNormal>(pointcloud1, "pointcloud1_v1", v1);//将点云添加到v1窗口
	//view->addPointCloud<pcl::PointNormal>(pointcloud2, "pointcloud2_v2", v2);//将点云添加到v2窗口
	//
	//view->setBackgroundColor(0.0, 0.0, 0.0, v1); //设着两个窗口的背景色
	//view->setBackgroundColor(0.0, 0.0, 0.0, v2); //设着两个窗口的背景色
	//
//	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> sources_cloud_color(normalcloud1, 250, 0, 0); //设置源点云的颜色为红色
//	//view->addPointCloud(normalcloud1, sources_cloud_color, "sources_cloud_v1", v1);//将点云添加到v1窗口
//	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> target_cloud_color(normalcloud2, 0, 250, 0);  //目标点云为绿色
//	//view->addPointCloud(normalcloud2, target_cloud_color, "target_cloud_v1", v1); //将点云添加到v1窗口
    //// 等待直到可视化窗口关闭
	//while (!view->wasStopped())
	//{
	//	view->spinOnce(100);
	//	boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	//}
}

//将PointNormal点云转换为PointXYZRGBNormal点云
void transform_normal2rgbnormal(pcl::PointCloud<pcl::PointNormal>::ConstPtr normalCloud, 
	                            pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr rgbnormalCloud)
{
	for (int irgb = 0; irgb<normalCloud->size(); irgb++)
	{
		pcl::PointXYZRGBNormal p;
		p.x = normalCloud->points[irgb].x;
		p.y = normalCloud->points[irgb].y;
		p.z = normalCloud->points[irgb].z;
		p.normal_x = normalCloud->points[irgb].normal_x;
		p.normal_y = normalCloud->points[irgb].normal_y;
		p.normal_z = normalCloud->points[irgb].normal_z;
		//r=255,g=255,b=255代表点为白色
		p.r = 255;
		p.g = 255;
		p.b = 255;
		rgbnormalCloud->points.push_back(p);
	}
}

//提取PointNormal云中的点XYZ存储为PointXYZ点云
pcl::PointCloud<pcl::PointXYZ>::Ptr extract_XYZcloud_PointNormal(pcl::PointCloud<pcl::PointNormal>::ConstPtr normcloud)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr XYZ_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	for (int ipoint = 0; ipoint<normcloud->size(); ipoint++)
	{
		pcl::PointXYZ p;
		p.x = normcloud->points[ipoint].x;
		p.y = normcloud->points[ipoint].y;
		p.z = normcloud->points[ipoint].z;
		XYZ_cloud->push_back(p);
	}
	return XYZ_cloud;
}

//提取PointXYZRGBNormal云中的点XYZ存储为PointXYZ点云
pcl::PointCloud<pcl::PointXYZ>::Ptr extract_XYZcloud_PointNormal(pcl::PointCloud<pcl::PointXYZRGBNormal>::ConstPtr rgbnormcloud)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr XYZ_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	for (int ipoint = 0; ipoint<rgbnormcloud->size(); ipoint++)
	{
		pcl::PointXYZ p;
		p.x = rgbnormcloud->points[ipoint].x;
		p.y = rgbnormcloud->points[ipoint].y;
		p.z = rgbnormcloud->points[ipoint].z;
		XYZ_cloud->push_back(p);
	}
	return XYZ_cloud;
}

//将PointNormal点云中的normal提取,存储为normal点云
pcl::PointCloud<pcl::Normal>::Ptr extract_Normal_PointNormal(pcl::PointCloud<pcl::PointNormal>::ConstPtr normcloud)
{
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normal(new pcl::PointCloud<pcl::Normal>);
	for (int inorm = 0; inorm<normcloud->size(); inorm++)
	{
		pcl::Normal norm;
		norm.normal_x = normcloud->points[inorm].normal_x;
		norm.normal_y = normcloud->points[inorm].normal_y;
		norm.normal_z = normcloud->points[inorm].normal_z;
		cloud_normal->push_back(norm);
	}
	return cloud_normal;
}

//将PointXYZRGBNormal点云中的normal提取,存储为normal点云
pcl::PointCloud<pcl::Normal>::Ptr extract_Normal_PointNormal(pcl::PointCloud<pcl::PointXYZRGBNormal>::ConstPtr rgbnormcloud)
{
	pcl::PointCloud<pcl::Normal>::Ptr rgbcloud_normal(new pcl::PointCloud<pcl::Normal>);
	for (int inorm = 0; inorm<rgbnormcloud->size(); inorm++)
	{
		pcl::Normal norm;
		norm.normal_x = rgbnormcloud->points[inorm].normal_x;
		norm.normal_y = rgbnormcloud->points[inorm].normal_y;
		norm.normal_z = rgbnormcloud->points[inorm].normal_z;
		rgbcloud_normal->push_back(norm);
	}
	return rgbcloud_normal;
}

//显示mormal点的法向量(按x,y,z查看)
void show_pointNormal(pcl::PointNormal normalpoint)
{
	std::cout << normalpoint.normal_x << " ";
	std::cout << normalpoint.normal_y << " ";
	std::cout << normalpoint.normal_z << std::endl;
}

//显示rgbmormal点的法向量(按x,y,z查看)
void show_rgbpointNormal(pcl::PointXYZRGBNormal rgbnormalpoint)
{
	std::cout << rgbnormalpoint.normal_x << " ";
	std::cout << rgbnormalpoint.normal_y << " ";
	std::cout << rgbnormalpoint.normal_z << std::endl;
}

#include <pcl/features/normal_3d.h>
#include <pcl\search\kdtree.h>
//计算xyz点云的法线,并将结果赋值到normalcloud点云
void compute_normal2cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,const float search_radius, 
	                      pcl::PointCloud<pcl::PointNormal>::Ptr normalcloud)
{
	//计算法线
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normcloud;
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
	//建立kdtree来进行近邻点集搜索
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	//为kdtree添加点运数据
	tree->setInputCloud(cloud);
	normcloud.setInputCloud(cloud);
	normcloud.setSearchMethod(tree);
	//点云法向计算时，需要所搜的近邻点大小
	normcloud.setKSearch(search_radius);
	//开始进行法向计算
	normcloud.compute(*cloud_normals);
	for (int i=0;i<cloud_normals->points.size();i++)
	{
		pcl::PointNormal point;
		point.x = cloud->points[i].x;
		point.y = cloud->points[i].y;
		point.z = cloud->points[i].z;
		point.normal_x = cloud_normals->points[i].normal_x;
		point.normal_y = cloud_normals->points[i].normal_y;
		point.normal_z = cloud_normals->points[i].normal_z;
		normalcloud->push_back(point);
	}
}

//绕x坐标轴旋转点云
void rota_cloud(pcl::PointCloud<pcl::PointNormal>::Ptr normalcloud_in, pcl::PointCloud<pcl::PointNormal>::Ptr normalcloud_out, float rota_angle)
{
	int num_points = normalcloud_in->size();
	float norm_sum = 0;
	for (int ipoint=0;ipoint<num_points; ipoint++)
	{
		pcl::PointNormal point;
		point.x = normalcloud_in->points[ipoint].x;
		point.y = (normalcloud_in->points[ipoint].y)*cos(rota_angle)-(normalcloud_in->points[ipoint].z)*sin(rota_angle);
		point.z = (normalcloud_in->points[ipoint].y)*sin(rota_angle) + (normalcloud_in->points[ipoint].z)*cos(rota_angle);
		norm_sum = sqrt(point.x*point.x + point.y*point.y + point.z*point.z);
		point.normal_x = point.x / norm_sum;
		point.normal_y = point.y / norm_sum;
		point.normal_z = point.z / norm_sum;
		normalcloud_out->push_back(point);
	}
}

void saprse_cloud(pcl::PointCloud<pcl::PointNormal>::ConstPtr normalcloud_orginal,
	              pcl::PointCloud<pcl::PointNormal>:: Ptr normalcloud_new,const int dismiss_num)
{
	int num_points = normalcloud_orginal->size();
	for (int ipoint=0;ipoint<num_points;ipoint++)
	{
		if (0==((ipoint+1)%dismiss_num))
		{
			;
		}
		else
		{
			normalcloud_new->push_back(normalcloud_orginal->points[ipoint]);
		}		
	}

	std::cout << "size for orginal:" << normalcloud_orginal->size() << std::endl;
	std::cout << "size for new:" << normalcloud_new->size() << std::endl;
}


