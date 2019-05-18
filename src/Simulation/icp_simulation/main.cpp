#include <iostream>
#include <osgWrapper\RenderService.h>
#include <osgWrapper\RenderView.h>

#include "Mesh.h"
#include "MeshLoader.h"
#include "MeshScaler.h"
#include "GemTransform.h"

#include "icp_scene.h"
#include <fstream>
using namespace LauncaGeometry;

int main(int argc, char* argv[])
{
	if (argc < 4)
		return EXIT_FAILURE;

	using namespace OSGWrapper;

	std::auto_ptr<Mesh> stable_mesh(MeshLoader::LoadFromFileName(argv[1]));
	std::auto_ptr<Mesh> patch_mesh(MeshLoader::LoadFromFileName(argv[2]));
	if (!stable_mesh.get() || !patch_mesh.get())
		return EXIT_FAILURE;

	std::string initial_matrix_file(argv[3]);

	GemTransform init_transform;
	GemTransform final_transform;
	std::fstream in;
	in.open(initial_matrix_file.c_str(), std::ios::in | std::ios::binary);
	if (in.is_open())
	{
		in.read((char*)init_transform.m_matrix, 16 * sizeof(float));
		in.read((char*)final_transform.m_matrix, 16 * sizeof(float));
	}
	in.close();

	{
		osg::ref_ptr<RenderView> view = new RenderView();
		osg::ref_ptr<ICPScene> scene = new ICPScene(*stable_mesh, *patch_mesh, init_transform);
		view->SetBackgroundColor(osg::Vec4(0.3f, 0.3f, 0.3f, 1.0f));
		view->setUpViewInWindow(50, 50, 1080, 720);
		view->SetCurrentScene(scene);

		RenderService::Instance().addView(view);
		RenderService::Instance().Run();
	}

	return EXIT_SUCCESS;
}