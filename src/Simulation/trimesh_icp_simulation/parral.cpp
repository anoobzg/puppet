#include "parral.h"
#include "load_trimeshes.h"
#include <iostream>
#include "load_calib.h"
#include "timestamp.h"
#include "projectionicp.h"

int test_single(trimesh::TriMesh* target, trimesh::TriMesh* source, int argc, char* argv[])
{
	std::cout << "test single" << std::endl;

	//trimesh::xform xf2_p;
	//
	//trimesh::timestamp t0 = trimesh::now();
	//trimesh::ProjectionICP icp(m_fx, m_fy, m_cx, m_cy);
	//icp.SetSource(source);
	//icp.SetTarget(target);
	//float err_p = icp.Do(xf2_p);
	//trimesh::timestamp t1 = trimesh::now();

	return EXIT_SUCCESS;
}

int test_parral(int argc, char* argv[])
{
	if (argc < 4) return EXIT_FAILURE;

	std::string cablifile(argv[2]);
	std::string source_dir(argv[3]);
	trimesh::CameraData camera_data;
	if (!load_camera_data_from_file(cablifile, camera_data))
		return EXIT_FAILURE;

	std::cout << "load meshes." << std::endl;
	std::vector<trimesh::TriMesh*> meshes;
	load_trimeshes(source_dir, meshes);

	size_t size = meshes.size();
	if (size < 2) return EXIT_FAILURE;

	if (argc >= 5 && !strcmp("single", argv[4]))
		return test_single(meshes.at(0), meshes.at(1), argc, argv);

	return EXIT_SUCCESS;
}