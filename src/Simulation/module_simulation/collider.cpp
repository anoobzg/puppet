#include "collider.h"
#include <osgWrapper\ColorIndexPicker.h>
#include "surface.h"

#include "CollideAlgrithm.h"
using namespace OSGWrapper;
Collider::Collider(Surface& surface, ColorIndexPicker& picker)
	:m_picker(picker), m_surface(surface)
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
	Mesh& mesh = m_surface.m_mesh;

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
	Mesh& mesh = m_surface.m_mesh;

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
	Mesh& mesh = m_surface.m_mesh;

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

bool Collider::QueryControlPoint(float* ray_position, float* ray_direction, unsigned& control_point_handle)
{
	const std::map<unsigned, ControlPoint*>& points = m_surface.m_points.Get();
	for (std::map<unsigned, ControlPoint*>::const_iterator it = points.begin(); it != points.end(); ++it)
	{
		ControlPoint* cp = (*it).second;

		osg::Vec3f point(cp->m_xyz.x, cp->m_xyz.y, cp->m_xyz.z);

		osg::Vec3f local_eye = osg::Vec3f(ray_position[0], ray_position[1], ray_position[2]);
		osg::Vec3f eye_to_point = point - local_eye;
		osg::Vec3f eye_to_center = osg::Vec3f(ray_direction[0], ray_direction[1], ray_direction[2]);

		float dot = eye_to_center * eye_to_point;
		if (dot <= 0.0f) continue;

		float slen = eye_to_point.length2();
		if (slen < dot * dot)
			continue;

		float d = sqrtf(slen - dot * dot);
		if (d > cp->m_radius)
			continue;

		control_point_handle = (*it).first;
		return true;
	}
	return false;
}