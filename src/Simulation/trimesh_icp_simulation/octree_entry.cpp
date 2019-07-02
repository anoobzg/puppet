#include "octree_entry.h"
#include <iostream>
#include "TriMesh.h"
#include <osgWrapper\RenderService.h>
#include <osgWrapper\RenderView.h>
#include "octreescene.h"
#include "octreework.h"
#include "octree.h"
#include "csvwriter.h"
#include "load_trimeshes.h"
#include "paraloctree.h"

int octree_entry(int argc, char* argv[])
{
	if (argc < 3) return EXIT_FAILURE;

	OctreeWork work;
	std::vector<trimesh::TriMesh*>& meshes = work.m_meshes;

	load_trimeshes(argv[2], meshes);
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
	if (argc < 4) return EXIT_FAILURE;

	std::vector<trimesh::TriMesh*> meshes;
	load_trimeshes(argv[2], meshes);

	if (meshes.size() == 0) return EXIT_FAILURE;

	Octree m_octree;
	//Octree m_quick_octree;
	ParalOctree m_paral_octree;

	CSVWriter writer;
	writer.PushHead("count");
	writer.PushHead("time1");
	writer.PushHead("time2");
	writer.PushHead("total1");
	writer.PushHead("total2");
	for (size_t i = 0; i < meshes.size(); ++i)
	{
		trimesh::TriMesh* mesh = meshes.at(i);
		mesh->normals.resize(mesh->vertices.size(), trimesh::vec3(1.0f, 0.0f, 1.0f));
		writer.PushData((double)mesh->vertices.size());
		
		if (i == 0)
		{
			m_octree.Initialize(mesh->bbox.center());
			m_paral_octree.Initialize(mesh->bbox.center());
		}

		writer.TickStart();
		m_octree.Insert(mesh->vertices, mesh->normals);
		writer.TickEnd();
		writer.TickStart();
		m_paral_octree.QuickInsert(mesh->vertices, mesh->normals);
		writer.TickEnd();

		writer.PushData((double)m_octree.m_current_point_index);
		writer.PushData((double)m_paral_octree.m_current_point_index);
	}

	std::string out_file(argv[3]);
	writer.Output(out_file);
	return EXIT_SUCCESS;
}