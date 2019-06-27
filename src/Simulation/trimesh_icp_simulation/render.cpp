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

void Render::OnFrame(PatchRenderData* data)
{
	task_runner()->PostTask(FROM_HERE, base::Bind(&Render::ShowPatch, base::Unretained(this), data));
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
	Convert(matrix, data->mesh->global);
	m_scene->ShowOneFrame(geometry, matrix);
	delete data;
}

void Render::ShowPatch(PatchRenderData* data)
{
	BuildPatch(data);
	osg::Matrixf matrix = osg::Matrixf::identity();
	Convert(matrix, data->xf);
	m_scene->UpdateMatrix(matrix, data->step == 0);
}

void Render::BuildPatch(PatchRenderData* data)
{
	m_scene->Lock();

	size_t exsit_size = m_geometries.size();
	std::vector<int> flags;
	if (exsit_size > 0) flags.resize(exsit_size, 0); //0: nothing,  1: update, 2: add

	std::vector<osg::ref_ptr<ChunkGeometry>> new_geometries;

	const std::vector<int> idxs = data->indices;
	const std::vector<trimesh::point>& points = data->points;
	const std::vector<trimesh::vec>& norms = data->normals;
	size_t input_size = idxs.size();
	bool has_color = false;
	float t = m_scene->GetTime();
	for (size_t i = 0; i < input_size; ++i)
	{
		int index = idxs.at(i);
		if (index < 0) continue;
		const trimesh::point& tp = points.at(i);
		const trimesh::vec& tn = norms.at(i);

		osg::Vec3f p = osg::Vec3f(tp[0], tp[1], tp[2]);
		osg::Vec3f n = osg::Vec3f(tn[0], tn[1], tn[2]);
		osg::Vec4f c = osg::Vec4f(0.8f, 0.8f, 0.8f, 1.0f);

		int chunk = index / ChunkVertexSize;
		int rindex = index % ChunkVertexSize;

		while (chunk >= (int)m_geometries.size())
		{
			ChunkGeometry* geometry = new ChunkGeometry();
			m_geometries.push_back(geometry);
			new_geometries.push_back(geometry);
		}

		ChunkGeometry* geometry = m_geometries.at(chunk);
		geometry->Update(rindex, p, n, c, t);
	}

	for (size_t i = 0; i < m_geometries.size(); ++i)
	{
		m_geometries.at(i)->Check();
	}

	for (size_t i = 0; i < new_geometries.size(); ++i)
		m_scene->AddPatch(new_geometries.at(i));

	bool first_frame = exsit_size == 0;
	if(first_frame) m_scene->UpdateCam();

	m_scene->Unlock();
}