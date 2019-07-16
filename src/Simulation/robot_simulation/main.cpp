#include <iostream>
#include <osgWrapper\RenderService.h>
#include <osgWrapper\RenderView.h>
#include "scene.h"
#include "Mesh.h"
#include "MeshLoader.h"
#include <osgWrapper\ArrayCreator.h>
#include <osgWrapper\GeometryCreator.h>

using namespace OSGWrapper;

int main(int argc, char* argv[])
{
	if (argc < 2) return EXIT_FAILURE;

	std::string file(argv[1]);
	std::auto_ptr<LauncaGeometry::Mesh> mesh(LauncaGeometry::MeshLoader::LoadFromFileName(file.c_str()));
	if (!mesh.get()) return EXIT_FAILURE;

	osg::Array* coord_array = OSGWrapper::ArrayCreator::CreateVec3Array(mesh->vertex_number, mesh->vertex_position);
	osg::Array* normal_array = OSGWrapper::ArrayCreator::CreateVec3Array(mesh->vertex_number, mesh->vertex_normal);
	osg::PrimitiveSet* primitive_set = OSGWrapper::ArrayCreator::CreatePrimitiveSet(osg::PrimitiveSet::TRIANGLES, 3 * mesh->triangle_number, mesh->triangle_index);

	osg::Geometry* geometry = OSGWrapper::GeometryCreator::CreateIndexAttributeGeometry(primitive_set, coord_array, normal_array);
	osg::ref_ptr<RenderView> view = new RenderView();
	osg::ref_ptr<RenderScene> scene = new Scene(geometry);
	view->SetBackgroundColor(osg::Vec4(0.0f, 0.0f, 0.0f, 1.0f));
	view->setUpViewInWindow(10, 10, 1080, 720);
	view->SetCurrentScene(scene);

	RenderService::Instance().addView(view);
	return RenderService::Instance().Run();
	return EXIT_SUCCESS;
}