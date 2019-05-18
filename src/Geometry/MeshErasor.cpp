#include "MeshErasor.h"
#include "Mesh.h"

#include <vector>
namespace LauncaGeometry
{
	Mesh* MeshErasor::Erase(Mesh& mesh, float* erase_flag, bool& is_empty)
	{
		is_empty = false;

		unsigned old_triangle_append_index = mesh.m_repair_triangle_append_index;
		unsigned old_vertex_append_index = mesh.m_repair_vertex_append_index;

		unsigned vertex_number = mesh.vertex_number;
		unsigned triangle_number = mesh.triangle_number;

		unsigned new_vertex_append_index = 0;
		unsigned new_triangle_append_index = 0;
		unsigned new_vertex_number = 0;
		unsigned new_triangle_number = 0;
		std::vector<bool> erase_vertex(vertex_number, false);
		std::vector<bool> erase_triangle(triangle_number, false);

		for (unsigned i = 0; i < vertex_number; ++i)
		{
			float flag = *(erase_flag + i);

			erase_vertex[i] = flag < 0.5f;
			if (!erase_vertex[i])
			{
				++new_vertex_number;
				if (i < old_vertex_append_index)
					++new_vertex_append_index;
			}
		}

		unsigned* triangle_index = mesh.triangle_index;
		for(unsigned i = 0; i < triangle_number; ++i)
		{
			bool this_tri_need_erase = false;

			unsigned tri_array[3];
			tri_array[0] = *triangle_index++;
			tri_array[1] = *triangle_index++;
			tri_array[2] = *triangle_index++;

			for(unsigned j = 0; j < 3; ++j)
			{
				unsigned vertex_index = tri_array[j];

				if(erase_vertex[vertex_index])
				{
					this_tri_need_erase = true;
					break;
				}
			}

			if(this_tri_need_erase)
				erase_triangle[i] = true;
			else
				++new_triangle_number;

			if(i < old_triangle_append_index && !this_tri_need_erase)
				++new_triangle_append_index;
		}

		if(new_vertex_number <= 0 || new_triangle_number <= 0)
		{
			is_empty = true;
			return 0;
		}

		if(new_vertex_number == vertex_number && new_triangle_number == triangle_number)
			return 0;

		Mesh* result_mesh = new Mesh;
		result_mesh->m_repair_triangle_append_index = new_triangle_append_index;
		result_mesh->m_repair_vertex_append_index = new_vertex_append_index;
		result_mesh->AllocateVertex(new_vertex_number);
		result_mesh->AllocateTriangle(new_triangle_number);

		float* normal_new = result_mesh->vertex_normal;
		float* coord_new = result_mesh->vertex_position;
		unsigned char* color_new = result_mesh->vertex_color;
		float* normal_old = mesh.vertex_normal;
		float* coord_old = mesh.vertex_position;
		unsigned char* color_old = mesh.vertex_color;

		std::vector<unsigned> vertex_new_index(vertex_number, 0);
		unsigned move_vertex_index = 0;
		for(unsigned i = 0; i < vertex_number; ++i)
		{
			if(!erase_vertex[i])
			{
				*normal_new++ = *normal_old++; *normal_new++ = *normal_old++; *normal_new++ = *normal_old++;
				*coord_new++ = *coord_old++; *coord_new++ = *coord_old++; *coord_new++ = *coord_old++;
				*color_new++ = *color_old++; *color_new++ = *color_old++; *color_new++ = *color_old++;
				vertex_new_index[i] = move_vertex_index++;
			}else
			{
				normal_old = normal_old + 3;
				coord_old = coord_old + 3;
				color_old = color_old + 3;
			}
		}

		unsigned* t_new = result_mesh->triangle_index;
		unsigned* t_old = mesh.triangle_index;
		for(unsigned i = 0; i < triangle_number; ++i)
		{
			if(!erase_triangle[i])
			{
				*t_new++ = vertex_new_index[*t_old++];
				*t_new++ = vertex_new_index[*t_old++];
				*t_new++ = vertex_new_index[*t_old++];
			}else
				t_old = t_old + 3;
		}

		return result_mesh;
	}
}