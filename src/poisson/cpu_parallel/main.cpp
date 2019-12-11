#include <iostream>
#include <PointCloudLoader.h>

#include <pcl/io/auto_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "TriMesh.h"

int main(int argc, const char* argv[])
{
	if (argc < 2)
	{
		std::cout << "Input Cloud File Missing." << std::endl;
		return EXIT_FAILURE;
	}

	std::string file(argv[1]);
	//pcl::PointCloud<pcl::PointXYZINormal> point_cloud;
	//if(pcl::io::loadPLYFile(file, point_cloud) != 0)
	std::unique_ptr<trimesh::TriMesh> mesh(trimesh::TriMesh::read(file));
	if(!mesh.get())
	{
		std::cout << "Can't Open PointCloud." << std::endl;
		return EXIT_FAILURE;
	}

	return EXIT_SUCCESS;
}