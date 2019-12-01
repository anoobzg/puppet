#pragma once
#include <osg\Vec3f>

namespace OSGWrapper
{
	class ColorIndexPicker;
}
using namespace OSGWrapper;

class MeshSegmentor;
class Collider
{
public:
	Collider(MeshSegmentor& segmentor, ColorIndexPicker& picker);
	~Collider();

	bool QuerySpacePoint(float x, float y, float* point);
	bool QueryPrimitive(float x, float y, unsigned& triangle_index);
	bool QueryVertex(float x, float y, unsigned i, unsigned& vertex_index); //i, triangle vertex index
	bool QueryVertex(float* ray_position, float* ray_direction, float x, float y, unsigned i, unsigned& vertex_index, float* point);
	osg::Vec3f GetVertex(unsigned handle);
private:
	MeshSegmentor& m_mesh_segmentor;
	ColorIndexPicker& m_picker;
};