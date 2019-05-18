#include "MeshVertexTraits.h"
#include "Mesh.h"
namespace LauncaGeometry
{
	void MeshVertexTraits::GetTriangleVertices(Mesh& mesh, float* x, float* y, float* z)
	{
		float* tx = x; float* ty = y; float* tz = z;
		for(unsigned i = 0; i < mesh.triangle_number; ++i)
		{
			unsigned* t_index = mesh.triangle_index + 3 * i;
			for(int j = 0; j < 3; ++j)
			{
				unsigned index = *t_index++;
				float* vertex = mesh.vertex_position + 3 * index;
				*tx++ = *vertex++;
				*ty++ = *vertex++;
				*tz++ = *vertex++;
			}
		}
	}

	void MeshVertexTraits::GetVertexCoord(Mesh& mesh, unsigned index, float& x, float& y, float& z)
	{
		float* position = mesh.vertex_position + 3 * index;
		x = *position++; y = *position++; z = *position++;
	}

	void MeshVertexTraits::GetVertices(Mesh& mesh, float* x, float* y, float* z, float* nx, float* ny, float* nz, bool skip_repaired)
	{
		unsigned size = skip_repaired ? mesh.m_repair_vertex_append_index : mesh.vertex_number;
		for(unsigned i = 0; i < size; ++i)
		{
			float* position = mesh.vertex_position + 3 * i;
			float* normal = mesh.vertex_normal + 3 * i;
			*x++ = *position++;
			*y++ = *position++;
			*z++ = *position++;
			*nx++ = *normal++;
			*ny++ = *normal++;
			*nz++ = *normal++;
		}
	}
}