#include <osgWrapper\SimpleRenderThread.h>
#include <osgWrapper\RenderService.h>
#include <osgWrapper\RenderView.h>

namespace OSGWrapper
{
	SimpleRenderThread::SimpleRenderThread(OSGWrapper::RenderScene* scene)
		:m_scene(scene)
	{

	}

	SimpleRenderThread::~SimpleRenderThread()
	{

	}

	void SimpleRenderThread::run()
	{
		if (!m_scene.valid())
			return;

		osg::ref_ptr<RenderView> view = new RenderView();
		view->SetBackgroundColor(osg::Vec4(0.3f, 0.3f, 0.3f, 1.0f));
		view->setUpViewInWindow(50, 50, 1080, 720);
		view->SetCurrentScene(m_scene);

		RenderService::Instance().addView(view);
		RenderService::Instance().Run();
	}

	int SimpleRenderThread::cancel()
	{
		RenderService::Instance().setDone(true);
		return Thread::cancel();
	}
}