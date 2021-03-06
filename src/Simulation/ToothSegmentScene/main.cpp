#include <iostream>
#include <osgWrapper\RenderService.h>
#include <osgWrapper\RenderView.h>

#include "Mesh.h"
#include "MeshLoader.h"
#include "MeshScaler.h"
#include "ToothSegmentScene.h"

using namespace LauncaGeometry;

void DeleteDegenerateTriangle(Mesh& mesh)
{
	std::set<unsigned> degenerateTriangleIndices;

	// Clear degenerate triangle
	for (unsigned i = 0; i < mesh.triangle_number; i++)
	{
		unsigned* triangleIndices = mesh.triangle_index + 3 * i;
		if (triangleIndices[0] == triangleIndices[1] ||
			triangleIndices[0] == triangleIndices[2] ||
			triangleIndices[1] == triangleIndices[2])
		{
			degenerateTriangleIndices.insert(i);
			continue;
		}
	}

	// Clear subMesh (opposite to main mesh)
	unsigned newSize = mesh.triangle_number - degenerateTriangleIndices.size();
	unsigned* newTriangleIndices = new unsigned[3 * newSize];

	unsigned offset = 0U;
	for (unsigned i = 0; i < mesh.triangle_number; i++)
	{
		if (degenerateTriangleIndices.count(i) > 0)
		{
			continue;
		}

		memcpy(newTriangleIndices + offset, mesh.triangle_index + 3 * i, sizeof(unsigned) * 3);
		offset += 3;
	}
	
	memcpy(mesh.triangle_index, newTriangleIndices, sizeof(unsigned) * newSize);
	mesh.triangle_number = newSize;

	delete[] newTriangleIndices;
	newTriangleIndices = nullptr;
}


int main(int argc, char* argv[])
{
	if (argc < 2)
		return EXIT_FAILURE;

	using namespace OSGWrapper;

	std::auto_ptr<Mesh> mesh(MeshLoader::LoadFromFileName(argv[1]));
	if (!mesh.get())
		return EXIT_FAILURE;

	DeleteDegenerateTriangle(*mesh);

	{
		osg::ref_ptr<RenderView> view = new RenderView();
		osg::ref_ptr<ToothSegmentScene> scene = new ToothSegmentScene(*mesh);
		view->SetBackgroundColor(osg::Vec4(0.3f, 0.3f, 0.3f, 1.0f));
		view->setUpViewInWindow(50, 50, 1080, 720);
		view->SetCurrentScene(scene);

		RenderService::Instance().addView(view);
		RenderService::Instance().Run();
	}

	return EXIT_SUCCESS;
}