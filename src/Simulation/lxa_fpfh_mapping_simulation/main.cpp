#include <iostream>
#include <osgWrapper\RenderService.h>
#include <osgWrapper\RenderView.h>

#include "mapping_scene.h"
#include "feature_object.h"
using namespace OSGWrapper;

int main(int argc, char* argv[])
{
	if (argc < 3)
		return EXIT_FAILURE;

	{
		std::auto_ptr<FeatureObject> fobject1(new FeatureObject(argv[1]));
		std::auto_ptr<FeatureObject> fobject2(new FeatureObject(argv[2]));

		//fobject2->Transform(0.0f, 0.0f, 10.0f);
		if (!fobject1->Valid() || !fobject2->Valid())
			return EXIT_FAILURE;

		osg::ref_ptr<RenderView> view = new RenderView();
		osg::ref_ptr<MappingScene> scene = new MappingScene(*fobject1, *fobject2);
		view->SetBackgroundColor(osg::Vec4(0.3f, 0.3f, 0.3f, 1.0f));
		view->setUpViewInWindow(50, 50, 1080, 720);
		view->SetCurrentScene(scene);

		RenderService::Instance().addView(view);
		RenderService::Instance().Run();
	}

	return EXIT_SUCCESS;
}