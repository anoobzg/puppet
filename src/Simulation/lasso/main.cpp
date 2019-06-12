#include <iostream>
#include <osgWrapper\RenderService.h>
#include <osgWrapper\RenderView.h>
#include "Mesh.h"
#include "MeshLoader.h"

#include "scene.h"
#include "lassoscene.h"
using namespace LauncaGeometry;

void Convert(Mesh& mesh, osg::Vec3Array*& coord_array, osg::Vec3Array*& normal_array)
{
	unsigned N = 40;
	unsigned vertex_num = mesh.vertex_number * N;
	coord_array = new osg::Vec3Array(vertex_num);
	normal_array = new osg::Vec3Array(vertex_num);
	for (unsigned i = 0; i < N; ++i)
	{
		float* p = mesh.vertex_position;
		float* n = mesh.vertex_normal;
		unsigned start_index = i * mesh.vertex_number;
		osg::Vec3f offset = osg::Vec3f((float)i * 1.0f, 0.0f, 0.0f);
		for (unsigned k = 0; k < mesh.vertex_number; ++k)
		{
			osg::Vec3f& ov = coord_array->at(start_index + k);
			osg::Vec3f& on = normal_array->at(start_index + k);
			ov.x() = *p++; ov.y() = *p++; ov.z() = *p++;
			ov += offset;
			on.x() = *n++; on.y() = *n++; on.z() = *n++;
		}
	}
}

int main(int argc, char* argv[])
{
	if (argc < 2)
		return EXIT_FAILURE;

	using namespace OSGWrapper;

	std::auto_ptr<Mesh> mesh(MeshLoader::LoadFromFileName(argv[1]));
	if (!mesh.get())
		return EXIT_FAILURE;

	osg::Vec3Array* coord_array = 0;
	osg::Vec3Array* normal_array = 0;
	Convert(*mesh, coord_array, normal_array);
	osg::ref_ptr<RenderView> view = new RenderView();
	//osg::ref_ptr<Scene> scene = new Scene(*mesh);
	osg::ref_ptr<LassoScene> scene = new LassoScene();
	scene->Set(coord_array, normal_array);
	view->SetBackgroundColor(osg::Vec4(0.3f, 0.3f, 0.3f, 1.0f));
	view->setUpViewInWindow(50, 50, 1080, 720);
	view->SetCurrentScene(scene);

	RenderService::Instance().addView(view);
	RenderService::Instance().setKeyEventSetsDone(0);
	return RenderService::Instance().Run();
}