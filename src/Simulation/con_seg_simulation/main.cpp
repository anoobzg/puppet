#include <iostream>
#include <osgWrapper\RenderService.h>
#include <osgWrapper\RenderView.h>

#include "Mesh.h"
#include "MeshLoader.h"
#include "MeshScaler.h"
#include "GemTransform.h"

#include "con_seg_scene.h"
#include <fstream>


using namespace LauncaGeometry;

int main(int argc, char* argv[])
{
	if (argc < 2)
		return EXIT_FAILURE;

	using namespace OSGWrapper;

	std::auto_ptr<Mesh> stable_mesh(MeshLoader::LoadFromFileName(argv[1]));
	if (!stable_mesh.get())
		return EXIT_FAILURE;

	{
		SegModule module;
		module.Setup(*stable_mesh);
		osg::ref_ptr<RenderView> view = new RenderView();
		osg::ref_ptr<ConSegScene> scene = new ConSegScene(*stable_mesh, module);
		view->SetBackgroundColor(osg::Vec4(0.3f, 0.3f, 0.3f, 1.0f));
		view->setUpViewInWindow(50, 50, 1080, 720);
		view->SetCurrentScene(scene);

		RenderService::Instance().addView(view);
		RenderService::Instance().Run();
	}

	return EXIT_SUCCESS;
}