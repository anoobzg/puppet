#pragma once

//point cloud header file
#include <pcl/point_types.h>                 //PCL对各种格式的点的支持头文件
#include <pcl/point_cloud.h>

typedef pcl::PointNormal Point;
typedef pcl::FPFHSignature33 FPFH;
//用到的命名空间
using namespace pcl;
using namespace std;
