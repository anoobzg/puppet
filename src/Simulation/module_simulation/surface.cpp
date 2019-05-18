#include "surface.h"
#include <memory>
#include "MeshVertexTraits.h"

Surface::Surface(Mesh& mesh)
	:m_mesh(mesh), m_callback(0), m_curvature_buffer(0)
{
	m_curvature_buffer = new float[m_mesh.vertex_number];
	memset(m_curvature_buffer, 0, sizeof(float) * m_mesh.vertex_number);

	m_path_algrithm.SetupGraph(mesh.vertex_number, mesh.vertex_position, mesh.triangle_number, mesh.triangle_index);
}

Surface::~Surface()
{
	if (m_curvature_buffer) delete[] m_curvature_buffer;
	m_curvature_buffer = 0;
}

void Surface::SetSurfaceTopoCallback(SurfaceTopoCallback* callback)
{
	m_callback = callback;
	if (m_callback) m_callback->ShowSurface(m_mesh);

	CalculateCurvature();
}

void Surface::CalculateCurvature()
{
	m_path_algrithm.getCurvatureData((char*)m_curvature_buffer);
	if(m_callback) m_callback->SurfaceCurvatureChanged(m_mesh.vertex_number, m_curvature_buffer);
}

void Surface::ShowGuassCurvature()
{
	m_path_algrithm.setCurvatureType(1);
	CalculateCurvature();
}

void Surface::ShowMeanCurvature()
{
	m_path_algrithm.setCurvatureType(0);
	CalculateCurvature();
}

ControlPoint* Surface::CreateControlPoint(unsigned vertex_handle, float* position)
{
	if (vertex_handle >= m_mesh.vertex_number)
		return nullptr;

	ControlPoint* point = m_points.Create(vertex_handle);
	if (point)
	{
		point->m_xyz.x = position[0];
		point->m_xyz.y = position[1];
		point->m_xyz.z = position[2];
	}
	return point;
}

void Surface::ModifyControlPoint(unsigned control_point_handle, unsigned vertex_handle, float* position)
{
	ControlPoint* control_point = m_points.Get(control_point_handle);
	if (control_point)
	{
		control_point->m_xyz.x = position[0];
		control_point->m_xyz.y = position[1];
		control_point->m_xyz.z = position[2];
		control_point->m_vertex_index = vertex_handle;
		if (m_callback) m_callback->ControlPointModified(*control_point);
	}
}

bool Surface::TryAddControlPoint(unsigned vertex_handle, float* position)
{
	ControlPoint* control_point = CreateControlPoint(vertex_handle, position);
	if (control_point == 0)
		return false;

	if (m_callback) m_callback->ControlPointAdded(*control_point);
	return true;
}

void Surface::DeleteControlPoint(unsigned control_point_handle)
{
	ControlPoint* control_point = m_points.Get(control_point_handle);

	if (control_point)
	{
		if (m_callback) m_callback->ControlPointDeleted(*control_point);
		m_points.Delete(control_point_handle);
	}
}

float Surface::GetProperRadius()
{
	return 1.0f;
}