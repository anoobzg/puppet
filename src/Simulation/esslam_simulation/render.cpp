#include "slam_data.h"
#include "render.h"
#include <base\bind.h>
#include <osgWrapper\ArrayCreator.h>
#include <osgWrapper\GeometryCreator.h>

osg::Array* CreateTrimeshArray(const std::vector<trimesh::vec3>& v)
{
	if (v.size() == 0) return 0;
	return OSGWrapper::ArrayCreator::CreateVec3Array(v.size(), (float*)&v.at(0));
}

Render::Render()
	:base::Thread("Render")
{
	m_scene = new SimulationScene();
}

Render::~Render()
{

}

void Render::OnFrame(esslam::PatchRenderData* data)
{
	task_runner()->PostTask(FROM_HERE, base::Bind(&Render::ShowPatch, base::Unretained(this), data));
}

void Render::OnFrameLocated(esslam::FrameData* data)
{
	task_runner()->PostTask(FROM_HERE, base::Bind(&Render::ShowCurrentFrame, base::Unretained(this), data));
}

void Render::OnNewPoints(esslam::NewAppendData* data)
{
	task_runner()->PostTask(FROM_HERE, base::Bind(&Render::AppendNewPoints, base::Unretained(this), data));
}

void Render::ShowCurrentFrame(esslam::FrameData* data)
{
	osg::Array* coord_array = CreateTrimeshArray(data->position);
	osg::Array* normal_array = CreateTrimeshArray(data->normals);
	osg::Array* distance_array = OSGWrapper::ArrayCreator::CreateFloatArray(data->distances.size(), (float*)(&data->distances.at(0)));
	osg::DrawArrays* draw_array = new osg::DrawArrays(GL_POINTS, 0, data->position.size());
	osg::Geometry* geometry = OSGWrapper::GeometryCreator::CreateIndexAttributeGeometry(draw_array, coord_array, normal_array, distance_array);

	osg::Matrixf matrix = osg::Matrixf::identity();
	Convert(matrix, data->xf);
	m_scene->ShowCurrentFrame(geometry, matrix);

	delete data;
}

void Render::AppendNewPoints(esslam::NewAppendData* data)
{
	m_scene->Lock();

	int CHUNK_SIZE = 100000;
	osg::Vec4f c = osg::Vec4f(134.0f/255.0f, 111.0f/255.0f, 93.0f/255.0f, 1.0f);
	int new_size = (int)data->position.size();
	for (int i = 0; i < new_size; ++i)
	{
		trimesh::vec3& v = data->position.at(i);
		trimesh::vec3& n = data->normals.at(i);
		if (!m_coord_array || m_coord_array->size() >= CHUNK_SIZE)
		{
			if(m_coord_array) m_coord_array->dirty();
			if(m_normal_array) m_normal_array->dirty();
			if(m_color_array) m_color_array->dirty();
			if (m_draw_arrays && m_coord_array)
			{
				m_draw_arrays->set(GL_POINTS, 0, m_coord_array->size());
				m_draw_arrays->dirty();
			}
			m_coord_array = new osg::Vec3Array();
			m_coord_array->reserve(CHUNK_SIZE);
			m_normal_array = new osg::Vec3Array();
			m_normal_array->reserve(CHUNK_SIZE);
			m_color_array = new osg::Vec4Array();
			m_color_array->reserve(CHUNK_SIZE);
			m_draw_arrays = new osg::DrawArrays(GL_POINTS);
			m_geometry = OSGWrapper::GeometryCreator::CreateIndexAttributeGeometry(m_draw_arrays, m_coord_array, m_normal_array, m_color_array);
			m_scene->AppendNewPoints(m_geometry);
		}
		
		m_coord_array->push_back(osg::Vec3f(v.x, v.y, v.z));
		m_normal_array->push_back(osg::Vec3f(n.x, n.y, n.z));
		m_color_array->push_back(c);
	}
	m_coord_array->dirty();
	m_normal_array->dirty();
	m_color_array->dirty();
	m_draw_arrays->set(GL_POINTS, 0, m_coord_array->size());
	m_draw_arrays->dirty();

	m_scene->Unlock();
	delete data;

	//osg::Array* coord_array = CreateTrimeshArray(data->position);
	//osg::Array* normal_array = CreateTrimeshArray(data->normals);
	//osg::Vec4Array* color_array = new osg::Vec4Array();
	//color_array->resize(data->position.size(), osg::Vec4f(1.0f, 1.0f, 1.0f, 1.0f));
	//osg::DrawArrays* draw_array = new osg::DrawArrays(GL_POINTS, 0, data->position.size());
	//osg::Geometry* geometry = OSGWrapper::GeometryCreator::CreateIndexAttributeGeometry(draw_array, coord_array, normal_array, color_array);

	//m_scene->AppendNewPoints(geometry);
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

void Render::Convert(osg::Matrixf& matrix, float* xf)
{
	for (int i = 0; i < 4; ++i)
		for (int j = 0; j < 4; ++j)
			matrix(i, j) = xf[4 * i + j];
}

void Render::ShowPatch(esslam::PatchRenderData* data)
{
	BuildPatch(data);
	osg::Matrixf matrix = osg::Matrixf::identity();
	Convert(matrix, data->xf);
	m_scene->UpdateMatrix(matrix, data->step == 0);

	delete data;
}

void Render::BuildPatch(esslam::PatchRenderData* data)
{
	m_scene->Lock();
	m_scene->UpdateCountText((int)data->indices.size());
	size_t exsit_size = m_geometries.size();
	std::vector<int> flags;
	if (exsit_size > 0) flags.resize(exsit_size, 0); //0: nothing,  1: update, 2: add

	std::vector<osg::ref_ptr<ChunkGeometry>> new_geometries;

	const std::vector<int> idxs = data->indices;
	const std::vector<float>& points = data->points;
	const std::vector<float>& norms = data->normals;
	size_t input_size = idxs.size();
	bool has_color = false;
	float t = m_scene->GetTime();
	for (size_t i = 0; i < input_size; ++i)
	{
		int index = idxs.at(i);
		if (index < 0) continue;
		const float* tp = &points.at(3 * i);
		const float* tn = &norms.at(3 * i);

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

	delete data;
}