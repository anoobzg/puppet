#include <iostream>
#include <osgWrapper\RenderService.h>
#include <osgWrapper\RenderView.h>
#include "scene.h"
#include "pscene.h"

extern float base_grid_size;
extern unsigned base_grid_vertex_number;
extern unsigned base_cell_number;

int main(int argc, char* argv[])
{
	if (argc < 4)
		return EXIT_FAILURE;

	base_grid_size = atof(argv[1]);
	base_grid_vertex_number = atoi(argv[2]);
	base_cell_number = base_grid_vertex_number - 1;

	unsigned simulation_mc = 0;
	if (argc >= 5) simulation_mc = (unsigned)atoi(argv[4]);
	using namespace OSGWrapper;

	osg::ref_ptr<RenderScene> scene;
	if (!simulation_mc)
		scene = new Scene(argc, argv);
	else
		scene = new PScene(argc, argv);

	osg::ref_ptr<RenderView> view = new RenderView();
	view->SetBackgroundColor(osg::Vec4(0.3f, 0.3f, 0.3f, 1.0f));
	view->setUpViewInWindow(50, 50, 1080, 720);
	view->SetCurrentScene(scene);

	RenderService::Instance().addView(view);
	return RenderService::Instance().Run();
}