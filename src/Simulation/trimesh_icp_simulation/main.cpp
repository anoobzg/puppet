#include <iostream>
#include <osgWrapper\RenderService.h>
#include <osgWrapper\RenderView.h>
#include "data.h"
#include "scene.h"
#include "icpscene.h"
#include "cmdicp.h"
#include "load_calib.h"
#include "compute_boundingbox.h"
#include "simulation_entry.h"
#include "octree_entry.h"
#include "parral.h"

using namespace OSGWrapper;

int main(int argc, char* argv[])
{
	if (argc >= 2 && !strcmp("simulation", argv[1]))
		return simulation_entry(argc, argv);

	if (argc >= 2 && !strcmp("vo", argv[1]))
		return vo(argc, argv);

	if (argc >= 2 && !strcmp("octree", argv[1]))
		return octree_entry(argc, argv);

	if (argc >= 2 && !strcmp("cmd_octree", argv[1]))
		return cmd_octree_entry(argc, argv);

	if (argc >= 2 && !strcmp("parral", argv[1]))
		return test_parral(argc, argv);

	if (argc >= 2 && !strcmp("testicp", argv[1]))
		return config_test_icp(argc, argv);

	if (argc < 4)
		return EXIT_FAILURE;

	std::string source(argv[2]);
	std::string target(argv[1]);
	std::string cablifile(argv[3]);
	trimesh::TriMesh* source_mesh = trimesh::TriMesh::read(source);
	trimesh::TriMesh* target_mesh = trimesh::TriMesh::read(target);

	if (!source_mesh || !target_mesh)
	{
		std::cout << "Source or Target file error." << std::endl;
		return EXIT_FAILURE;
	}

	trimesh::CameraData camera_data;
	if (!load_camera_data_from_file(cablifile, camera_data))
	{
		std::cout << "Cabli Data Error." << std::endl;
		return EXIT_FAILURE;
	}

	ComputeBoundingbox(source_mesh->vertices, source_mesh->bbox);
	ComputeBoundingbox(target_mesh->vertices, target_mesh->bbox);

	if (argc >= 5 && !strcmp("cmd", argv[4]))
	{
		cmd_test_icp(source_mesh, target_mesh,camera_data);
		return EXIT_SUCCESS;
	}

	if (argc >= 7 && !strcmp("analysis", argv[4]))
	{
		std::string error_file(argv[5]);
		std::string time_file(argv[6]);
		cmd_analysis_icp(source_mesh, target_mesh, camera_data, error_file, time_file);
		return EXIT_SUCCESS;
	}

	trimesh::xform init_xf;
	if (argc >= 5)
	{
		std::string matrix_file(argv[4]);
		std::fstream in;
		in.open(matrix_file.c_str(), std::ios::binary | std::ios::in);
		if (in.is_open())
		{
			in.read((char*)init_xf.data(), sizeof(double) * 16);
		}
		in.close();
	}
	source_mesh->global = init_xf;
	osg::ref_ptr<RenderView> view = new RenderView();
	osg::ref_ptr<RenderScene> scene = new ICPScene(camera_data, *source_mesh, *target_mesh);
	view->SetBackgroundColor(osg::Vec4(0.3f, 0.3f, 0.3f, 1.0f));
	view->setUpViewInWindow(0, 0, 1920, 1080);
	view->SetCurrentScene(scene);

	RenderService::Instance().addView(view);
	return RenderService::Instance().Run();
}
//
//int main(int argc, char* argv[])
//{
//	if (argc < 2)
//		return EXIT_FAILURE;
//
//	std::string dir(argv[1]);
//	Data data;
//	data.Load(dir);
//
//	osg::ref_ptr<RenderView> view = new RenderView();
//	osg::ref_ptr<RenderScene> scene = new Scene(data);
//	view->SetBackgroundColor(osg::Vec4(0.3f, 0.3f, 0.3f, 1.0f));
//	view->setUpViewInWindow(50, 50, 1080, 720);
//	view->SetCurrentScene(scene);
//
//	RenderService::Instance().addView(view);
//	RenderService::Instance().setKeyEventSetsDone(0);
//	return RenderService::Instance().Run();
//}