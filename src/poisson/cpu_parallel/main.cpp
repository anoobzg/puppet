#include <iostream>

#include "point_loader.h"
#include "validator.h"
#include "parallel_data_octree.h"

int main(int argc, const char* argv[])
{
	if (argc < 2)
	{
		std::cout << "Input Cloud File Missing." << std::endl;
		return EXIT_FAILURE;
	}

	std::vector<trimesh::vec3> positions;
	std::vector<trimesh::vec3> normals;
	PointLoader::LoadFromFile(argv[1], positions, normals);

	size_t psize = positions.size();
	size_t nsize = normals.size();
	if ((psize == 0) || (nsize == 0) || (psize != nsize))
	{
		std::cout << "Input Cloud Error. psize "<<psize<<" nsize "<<nsize << std::endl;
		return EXIT_FAILURE;
	}

	Validator::Validate(positions, normals);

	ParallelDataOctree octree;
	octree.BuildFromPoints(positions, normals);
	return EXIT_SUCCESS;
}