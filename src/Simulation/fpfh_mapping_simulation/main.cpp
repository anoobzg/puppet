#include <iostream>
#include <osgWrapper\RenderService.h>
#include <osgWrapper\RenderView.h>

#include "Mesh.h"
#include "MeshLoader.h"
#include "MeshScaler.h"
#include "feature_object.h"
#include "mapping_scene.h"

using namespace LauncaGeometry;

int main(int argc, char* argv[])
{
	if (argc < 2)
		return EXIT_FAILURE;

	std::string tool_name(argv[1]);
	if (!strcmp(tool_name.c_str(), "visual"))
	{
		if (argc < 4)
			return EXIT_FAILURE;

		using namespace OSGWrapper;

		std::auto_ptr<Mesh> mesh1(MeshLoader::LoadFromFileName(argv[2]));
		std::auto_ptr<Mesh> mesh2(MeshLoader::LoadFromFileName(argv[3]));
		if (!mesh1.get() || !mesh2.get())
			return EXIT_FAILURE;

		if (argc == 5)
		{
			MeshScaler::Scale(0.001f, 0.001f, 0.001f, *mesh1);
			MeshScaler::Scale(0.001f, 0.001f, 0.001f, *mesh2);
		}
		{
			std::auto_ptr<FeatureObject> fobject1(new FeatureObject(*mesh1));
			std::auto_ptr<FeatureObject> fobject2(new FeatureObject(*mesh2));

			osg::ref_ptr<RenderView> view = new RenderView();
			osg::ref_ptr<MappingScene> scene = new MappingScene(*fobject1, *fobject2);
			view->SetBackgroundColor(osg::Vec4(0.3f, 0.3f, 0.3f, 1.0f));
			view->setUpViewInWindow(50, 50, 1080, 720);
			view->SetCurrentScene(scene);

			RenderService::Instance().addView(view);
			RenderService::Instance().Run();
		}
	}
	else if (!strcmp(tool_name.c_str(), "split"))
	{
		if (argc < 4)
			return EXIT_FAILURE;

		std::auto_ptr<Mesh> mesh(MeshLoader::LoadFromFileName(argv[2]));
		float chunk_size = atof(argv[3]);
	}

	return EXIT_SUCCESS;
}