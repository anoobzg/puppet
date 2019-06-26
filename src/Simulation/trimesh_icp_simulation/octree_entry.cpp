#include "octree_entry.h"
#include <iostream>
#include "TriMesh.h"
#include <boost\format.hpp>
#include <osgWrapper\RenderService.h>
#include <osgWrapper\RenderView.h>
#include "octreescene.h"
#include "octreework.h"

void load_trimeshes(int argc, char* argv[], std::vector<trimesh::TriMesh*>& meshes)
{
	std::string directory = argv[2];
	int index = 0;
	while (true)
	{
		std::string file = directory + "//" +
			boost::str(boost::format("%d.ply") % index);
		trimesh::TriMesh* mesh = trimesh::TriMesh::read(file);
		if (mesh) meshes.push_back(mesh);
		else break;

		++index;
	}
}

int octree_entry(int argc, char* argv[])
{
	if (argc < 3) return EXIT_FAILURE;

	OctreeWork work;
	std::vector<trimesh::TriMesh*>& meshes = work.m_meshes;

	load_trimeshes(argc, argv, meshes);
	if (meshes.size() == 0) return EXIT_FAILURE;

	osg::ref_ptr<OSGWrapper::RenderView> view = new OSGWrapper::RenderView();
	osg::ref_ptr<OSGWrapper::RenderScene> scene = new OctreeScene(work);
	view->SetBackgroundColor(osg::Vec4(0.3f, 0.3f, 0.3f, 1.0f));
	view->setUpViewInWindow(0, 0, 1920, 1080);
	view->SetCurrentScene(scene);

	OSGWrapper::RenderService::Instance().addView(view);
	return OSGWrapper::RenderService::Instance().Run();;
}

int cmd_octree_entry(int argc, char* argv[])
{
	if (argc < 3) return EXIT_FAILURE;

	std::vector<trimesh::TriMesh*> meshes;
	load_trimeshes(argc, argv, meshes);

	if (meshes.size() == 0) return EXIT_FAILURE;

	return EXIT_SUCCESS;
}