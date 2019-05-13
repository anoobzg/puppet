#include <osgWrapper\SimMain_Ex.h>
#include <osgWrapper\RenderService.h>

int RunScene(RenderScene* scene)
{
	if (!scene)
		return EXIT_FAILURE;

	RenderView *view = (RenderView*)RenderService::Instance().getView(0);
	view->SetCurrentScene(scene);

	return RenderService::Instance().Run();
}


int main(int argc, char* argv[])
{
	osg::ref_ptr<RenderView> view = new RenderView();
	view->SetBackgroundColor(osg::Vec4(0.5f, 0.5f, 0.5f, 1.0f));
	view->setUpViewInWindow(50, 50, 1080, 720);

	RenderService::Instance().addView(view);

	return RunRenderScene(argc, argv);
}
