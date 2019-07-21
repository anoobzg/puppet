#include <iostream>
#include <osgWrapper\RenderService.h>
#include <osgWrapper\RenderView.h>
#include "scene.h"

using namespace OSGWrapper;

int main(int argc, char* argv[])
{
	osg::ref_ptr<RenderView> view = new RenderView();
	osg::ref_ptr<Scene> scene = new Scene();
	scene->LoadFrom(argc, argv);
	view->SetBackgroundColor(osg::Vec4(0.0f, 0.0f, 0.0f, 1.0f));
	view->setUpViewInWindow(10, 10, 1080, 720);
	view->SetCurrentScene(scene);

	RenderService::Instance().addView(view);
	return RenderService::Instance().Run();
	return EXIT_SUCCESS;
}