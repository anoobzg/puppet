#include <iostream>
#include <memory>

#include "interface.h"
#include "PointCloudLoader.h"
#include "octree.h"
#include "solver.h"

int main(int argc, char* argv[])
{
	if (argc < 2)
	{
		std::cout << "Input Cloud File Missing." << std::endl;
		return EXIT_FAILURE;
	}

	std::string file(argv[1]);
	std::auto_ptr<PointCloud> raw_point(PointCloudLoader::LoadFromFileName(file.c_str()));
	if (!raw_point.get())
	{
		std::cout << "Can't Open PointCloud." << std::endl;
		return EXIT_FAILURE;
	}

	std::vector<point> points;
	build_raw_points(*raw_point, points);

	vec3 recon_max;
	vec3 recon_min; 
	unsigned depth = 0;
	float grid_size = 80.0f;
	build_octree_parameters(points, recon_max, recon_min, depth, grid_size);
	depth = 7;

	std::auto_ptr<Solver> solver(new Solver(depth));

	std::auto_ptr<Octree> octree(new Octree(depth));
	octree->Build(points, recon_max, recon_min);

	solver->Solve(*octree);
	system("pause");
	return EXIT_SUCCESS;
}