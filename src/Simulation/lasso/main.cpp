#include <iostream>
#include <osgWrapper\RenderService.h>
#include <osgWrapper\RenderView.h>
#include "Mesh.h"
#include "MeshLoader.h"

#include "scene.h"

using namespace LauncaGeometry;
int main(int argc, char* argv[])
{
	if (argc < 2)
		return EXIT_FAILURE;

	using namespace OSGWrapper;

	std::auto_ptr<Mesh> mesh(MeshLoader::LoadFromFileName(argv[1]));
	if (!mesh.get())
		return EXIT_FAILURE;

	osg::ref_ptr<RenderView> view = new RenderView();
	osg::ref_ptr<Scene> scene = new Scene(*mesh);
	view->SetBackgroundColor(osg::Vec4(0.3f, 0.3f, 0.3f, 1.0f));
	view->setUpViewInWindow(50, 50, 1080, 720);
	view->SetCurrentScene(scene);

	RenderService::Instance().addView(view);
	RenderService::Instance().setKeyEventSetsDone(0);
	return RenderService::Instance().Run();
}