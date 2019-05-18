#include "RepairedRemover.h"
#include "Mesh.h"

#include <string.h>
namespace LauncaGeometry
{
	Mesh* RepairedRemover::Remove(Mesh& mesh)
	{
		Mesh* result = 0;

		int vertex_repair_start = mesh.m_repair_vertex_append_index;
		int triangle_repair_start = mesh.m_repair_triangle_append_index;

		if(((unsigned)vertex_repair_start < mesh.vertex_number)
			|| ((unsigned)triangle_repair_start < mesh.vertex_number))
		{
			result = new Mesh();
			result->AllocateVertex((unsigned)vertex_repair_start);
			result->AllocateTriangle((unsigned)triangle_repair_start);

			memcpy(result->vertex_position, mesh.vertex_position, 3 * result->vertex_number * sizeof(float));
			memcpy(result->vertex_normal, mesh.vertex_normal, 3 * result->vertex_number * sizeof(float));
			memcpy(result->vertex_color, mesh.vertex_color, 3 * result->vertex_number * sizeof(unsigned char));
			memcpy(result->triangle_index, mesh.triangle_index, 3 * result->triangle_number * sizeof(unsigned));
		}

		return result;
	}
}