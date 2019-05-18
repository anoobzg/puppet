#include <iostream>
#include <osgWrapper\RenderService.h>
#include <osgWrapper\RenderView.h>
#include "shortest_path_scene.h"

int main(int argc, char* argv[])
{
	if (argc < 2)
		return EXIT_FAILURE;

	using namespace OSGWrapper;

	osg::ref_ptr<RenderView> view = new RenderView();
	osg::ref_ptr<Scene> scene = new Scene(argc, argv, view);
	view->SetBackgroundColor(osg::Vec4(0.3f, 0.3f, 0.3f, 1.0f));
	view->setUpViewInWindow(50, 50, 1080, 720);
	view->SetCurrentScene(scene);

	RenderService::Instance().addView(view);
	return RenderService::Instance().Run();
}