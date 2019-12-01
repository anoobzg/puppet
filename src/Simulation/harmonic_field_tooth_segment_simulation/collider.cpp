#include "collider.h"
#include <osgWrapper\ColorIndexPicker.h>
#include "mesh_segmentor.h"

#include "CollideAlgrithm.h"
using namespace OSGWrapper;
Collider::Collider(MeshSegmentor& segmentor, ColorIndexPicker& picker)
	:m_picker(picker), m_mesh_segmentor(segmentor)
{

}

Collider::~Collider()
{

}

bool Collider::QuerySpacePoint(float x, float y, float* point)
{
	return false;
}

bool Collider::QueryPrimitive(float x, float y, unsigned& triangle_index)
{
	Mesh& mesh = m_mesh_segmentor.m_mesh;
	
	unsigned mesh_id = 0;
	unsigned primitive_id = 0;
	m_picker.Pick(x, y, mesh_id, primitive_id);
	
	if (primitive_id > 0 && primitive_id <= mesh.triangle_number)
	{
		triangle_index = primitive_id - 1;
		return true;
	}
	return false;
}

bool Collider::QueryVertex(float x, float y, unsigned i, unsigned& vertex_index)
{
	Mesh& mesh = m_mesh_segmentor.m_mesh;
	
	unsigned triangle_index = -1;
	bool result = QueryPrimitive(x, y, triangle_index);
	
	if (result)
	{
		unsigned* index = mesh.triangle_index + 3 * triangle_index + i;
		vertex_index = *index;
	}
	return result;
}

bool Collider::QueryVertex(float* ray_position, float* ray_direction, float x, float y, unsigned i, unsigned& vertex_index, float* point)
{
	Mesh& mesh = m_mesh_segmentor.m_mesh;
	
	unsigned triangle_index = -1;
	bool result = QueryPrimitive(x, y, triangle_index);
	
	if (result)
	{
		unsigned* index = mesh.triangle_index + 3 * triangle_index + i;
		vertex_index = *index;
	
		unsigned index0 = *(mesh.triangle_index + 3 * triangle_index);
		unsigned index1 = *(mesh.triangle_index + 3 * triangle_index + 1);
		unsigned index2 = *(mesh.triangle_index + 3 * triangle_index + 2);
	
		float* v0 = mesh.vertex_position + 3 * index0;
		float* v1 = mesh.vertex_position + 3 * index1;
		float* v2 = mesh.vertex_position + 3 * index2;
	
		float uvw[3] = { 0.0f };
		ColliderAlgrithm::RayCollideTriangle(ray_position, ray_direction, v0, v1, v2, uvw, point);
	}
	return result;
}

osg::Vec3f Collider::GetVertex(unsigned handle)
{
	Mesh& mesh = m_mesh_segmentor.m_mesh;

	float* v0 = mesh.vertex_position + 3 * handle;
	osg::Vec3f p(*v0, *(v0 + 1), *(v0 + 2));
	return p;
}
