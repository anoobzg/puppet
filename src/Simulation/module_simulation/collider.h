#pragma once
#include <osg\Vec3f>

namespace OSGWrapper
{
	class ColorIndexPicker;
}
using namespace OSGWrapper;

class Surface;
class Collider
{
public:
	Collider(Surface& surface, ColorIndexPicker& picker);
	~Collider();

	bool QuerySpacePoint(float x, float y, float* point);
	bool QueryPrimitive(float x, float y, unsigned& triangle_index);
	bool QueryVertex(float x, float y, unsigned i, unsigned& vertex_index); //i, triangle vertex index
	bool QueryVertex(float* ray_position, float* ray_direction, float x, float y, unsigned i, unsigned& vertex_index, float* point);
	bool QueryControlPoint(float* ray_position, float* ray_direction, unsigned& control_point_handle);
private:
	Surface& m_surface;
	ColorIndexPicker& m_picker;
};