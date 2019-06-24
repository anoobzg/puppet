#include "render.h"
#include <base\bind.h>
#include "trimeshgeometrycreator.h"

Render::Render()
	:base::Thread("Render")
{
	m_scene = new SimulationScene();
}

Render::~Render()
{

}

void Render::OnFrame(RenderData* data)
{
	task_runner()->PostTask(FROM_HERE, base::Bind(&Render::ShowOneFrame, base::Unretained(this), data));
}

void Render::StartRender()
{
	Start();
}

void Render::StopRender()
{
	Stop();
}

SimulationScene* Render::GetScene()
{
	return m_scene;
}

void Render::Convert(osg::Matrixf& matrix, const trimesh::xform& xf)
{
	for (int i = 0; i < 4; ++i)
		for (int j = 0; j < 4; ++j)
			matrix(i, j) = xf(j, i);
}

void Render::ShowOneFrame(RenderData* data)
{
	osg::Geometry* geometry = Create(data->mesh.get());
	osg::Matrixf matrix = osg::Matrixf::identity();
	Convert(matrix, data->xf);
	m_scene->ShowOneFrame(geometry, matrix);
	delete data;
}