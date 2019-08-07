#include <iostream>
#include <osgWrapper/RenderService.h>
#include <osgWrapper/RenderView.h>

#include "es_render.h"
#include "es_callback.h"
int main(int argc, char* argv[])
{
	using namespace OSGWrapper;

	osg::ref_ptr<RenderView> view = new RenderView();
	view->SetBackgroundColor(osg::Vec4(0.0f, 0.0f, 0.0f, 1.0f));
	view->setUpViewInWindow(30, 30, 1580, 920);

	ESRender render(*view);
	std::unique_ptr<EsCallback> callback(new EsCallback(render));
	render.SetUICallback(callback.get());

	RenderService::Instance().addView(view);
	int result = RenderService::Instance().Run();
	render.SetUICallback(NULL);

	return result;
}