#include <iostream>
#include <osgWrapper\RenderService.h>
#include <osgWrapper\RenderView.h>
#include "scene.h"
#include "Mesh.h"
#include "MeshLoader.h"
#include <osgWrapper\ArrayCreator.h>
#include <osgWrapper\GeometryCreator.h>
#include <osgWrapper\TextureManager.h>
#include "load_obj.h"

using namespace OSGWrapper;

int main(int argc, char* argv[])
{
	if (argc < 3) return EXIT_FAILURE;

	std::string file(argv[1]);
	std::string texture_file(argv[2]);

	osg::Geometry* geometry = LoadObj(file);

	OSGWrapper::UnionCoord* coord = 0;
	osg::Texture2D* texture = OSGWrapper::D2TextureManager::Instance().Get(texture_file, coord, true);

	osg::ref_ptr<RenderView> view = new RenderView();
	osg::ref_ptr<RenderScene> scene = new Scene(geometry, texture);
	view->SetBackgroundColor(osg::Vec4(0.0f, 0.0f, 0.0f, 1.0f));
	view->setUpViewInWindow(10, 10, 1080, 720);
	view->SetCurrentScene(scene);

	RenderService::Instance().addView(view);
	return RenderService::Instance().Run();
	return EXIT_SUCCESS;
}