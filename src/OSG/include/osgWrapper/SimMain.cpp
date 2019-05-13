#include <osgWrapper\SimMain.h>
#include <osgWrapper\RenderService.h>

namespace OSGWrapper
{
	int SimMain(int argc, char* argv[])
	{
		osg::ref_ptr<RenderScene> scene = CreateRenderScene(argc, argv);
		if (!scene.valid())
			return EXIT_FAILURE;

		osg::ref_ptr<RenderView> view = new RenderView();
		view->SetBackgroundColor(osg::Vec4(0.3f, 0.3f, 0.3f, 1.0f));
		view->setUpViewInWindow(50, 50, 1080, 720);
		view->SetCurrentScene(scene);

		RenderService::Instance().addView(view);
		return RenderService::Instance().Run();
	}
}