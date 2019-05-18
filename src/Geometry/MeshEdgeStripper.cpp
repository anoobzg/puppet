#include "MeshEdgeStripper.h"
#include "Mesh.h"
#include <map>
#include <algorithm>

namespace LauncaGeometry
{
	MeshEdgeStripper::MeshEdgeStripper(Mesh& mesh)
		:m_mesh(mesh)
	{
	}

	MeshEdgeStripper::~MeshEdgeStripper()
	{
	}

	bool MeshEdgeStripper::CheckValid(std::vector<unsigned>& edges)
	{
		if (m_mesh.vertex_number == 0 || m_mesh.triangle_number == 0 || edges.size() == 0)
			return true;

		//Check edge validation
		bool has_invalid_vertex = false;
		for (size_t i = 0; i < edges.size(); ++i)
		{
			unsigned index = edges[i];
			if (index >= m_mesh.vertex_number)
			{
				has_invalid_vertex = true;
				break;
			}
		}

		if (has_invalid_vertex) return false;
		return true;
	}

	bool MeshEdgeStripper::BuildUpData(std::vector<unsigned>& edges)
	{
		//collect triangles types
		m_triangles_type_indicator.resize(m_mesh.triangle_number);
		m_vertex_mapping.resize(m_mesh.vertex_number, -1);

		bool bMapped = false;
		for (size_t i = 0; i < edges.size(); ++i)
		{
			unsigned index = edges[i];
			m_vertex_mapping[index] = (int)i;
			bMapped = true;
		}
		if (!bMapped)
		{
			return false;
		}

		unsigned type3triCount = edges.size() - 1;
		unsigned* type3triIndices = new unsigned[3 * type3triCount];
		if (!type3triIndices)
		{
			return false;
		}

		unsigned type2triCount = edges.size() - 1;
		unsigned* type2triIndices = new unsigned[2 * type2triCount];
		if (!type2triIndices)
		{
			return false;
		}

		for (unsigned i = 0; i < type3triCount; i++)
		{
			type3triIndices[3 * i] = edges[i % edges.size()];
			type3triIndices[3 * i + 1] = edges[(i + 1) % edges.size()];
			type3triIndices[3 * i + 2] = edges[(i + 2) % edges.size()];
		}

		for (unsigned i = 0; i < type2triCount; i++)
		{
			type2triIndices[2 * i] = edges[i % edges.size()];
			type2triIndices[2 * i + 1] = edges[(i + 1) % edges.size()];
		}

		auto triangleMatch = [](unsigned* triangleIndices, unsigned* matchIndices, unsigned matchCount)
		{
			if (matchCount > 3 || 1 > matchCount)
			{
				return false;
			}

			if (matchIndices[0] == matchIndices[1])
			{
				return false;
			}

			if (matchCount == 3)
			{
				if (matchIndices[0] == matchIndices[2] ||
					matchIndices[1] == matchIndices[2])
				{
					return false;
				}
			}

			int nEqualCount = 0;

			for (int i = 0; i < matchCount; i++)
			{
				if (triangleIndices[0] == matchIndices[i] ||
					triangleIndices[1] == matchIndices[i] ||
					triangleIndices[2] == matchIndices[i])
				{
					nEqualCount++;
				}
			}

			return nEqualCount == matchCount;
		};

		for (unsigned i = 0; i < m_mesh.triangle_number; ++i)
		{
			triangle_type_info& info = m_triangles_type_indicator[i];
			unsigned* triangle_index = m_mesh.triangle_index + 3 * i;
			unsigned count = 0;

			for (unsigned j = 0; j < 3; ++j)
			{
				unsigned index = *(triangle_index + j);
				if (m_vertex_mapping[index] >= 0)
				{
					count++;
				}
			}

			if (count > 1)
			{
				bool bMatched = false;
				unsigned matchIndices[3] = {0, 0, 0};

				// Match type3 triangle
				for (unsigned i = 0; i < type3triCount; i++)
				{
					if (triangleMatch(triangle_index, type3triIndices + 3 * i, 3))
					{
						bMatched = true;
						matchIndices[0] = *(type3triIndices + 3 * i);
						matchIndices[1] = *(type3triIndices + 3 * i + 1);
						matchIndices[2] = *(type3triIndices + 3 * i + 2);
						count = 3;
						break;
					}
				}

				if (!bMatched)
				{
					// Exception: if type2 fit count == 2 means this triangle is type3

					int nType2Count = 0;
					for (unsigned i = 0; i < type2triCount; i++)
					{
						if (triangleMatch(triangle_index, type2triIndices + 2 * i, 2))
						{
							nType2Count++;
							for (int m = 0; m < 2; m++)
							{
								bool bExist = false;

								for (int n = 0; n < 3; n++)
								{
									if (*(type2triIndices + 2 * i + m) == matchIndices[n])
									{
										bExist = true;
									}
								}

								if (bExist)
								{
									continue;
								}

								bool bAdded = false;
								for (int n = 0; n < 3; n++)
								{
									if (matchIndices[n] == 0)
									{
										bAdded = true;
										matchIndices[n] = *(type2triIndices + 2 * i + m);
										break;
									}
								}

								if (!bAdded)
								{
									return false;
								}
							}
						}
					}

					if (nType2Count == 2)
					{
						bMatched = true;
						if (matchIndices[2] != 0)
						{
							count = 3;
						}
						else
						{
							count = 2;
						}
					}

					else if (nType2Count == 1)
					{
						bMatched = true;
						count = 2;
					}

					else 
					{
					}
				}

				if (bMatched)
				{
					int firstMatchIndex = -1;
					for (unsigned i = 0; i < 3; i++)
					{
						if (*(triangle_index + i) == matchIndices[0])
						{
							firstMatchIndex = i;
							count = (*(triangle_index + ((i + 1) % 3)) == matchIndices[1]) ?  0: count;
						}
					}

					if (count == 3)
					{
						info.i[0] = 0;
						info.i[1] = 0;
						info.i[2] = 0;
					}
					else if (count == 2)
					{
						for (int j = 0; j < 3; j++)
						{
							if ((*(triangle_index + j) == matchIndices[0]) ||
								(*(triangle_index + j) == matchIndices[1]))
							{
								info.i[j] = 0;
							}
						}
					}
				}
				else
				{
					count = 0;
				}
			}

			else
			{
				// Ignore type1 triangle
				count = 0;
			}

			info.type = count;
		}

		if (type2triCount)
		{
			delete[] type2triIndices;
		}

		if (type3triIndices)
		{
			delete[] type3triIndices;
		}

		return true;
	}

	Mesh* MeshEdgeStripper::Do(std::vector<unsigned>& edges, std::vector<unsigned>& triangles)
	{
		bool result = CheckValid(edges);
		if (!result) 
			return 0;

		m_triangles_type_indicator.clear();
		m_vertex_mapping.clear();

		result = BuildUpData(edges);
		if (!result) 
			return 0;

		unsigned type1_count = 0;
		unsigned type2_count = 0;
		for (size_t i = 0; i < m_triangles_type_indicator.size(); ++i)
		{
			triangle_type_info& info = m_triangles_type_indicator[i];
			if (info.type == 1) ++type1_count;
			if (info.type == 2) ++type2_count;
		}

		if (type1_count == 0 && type2_count == 0)
			return 0;

		//result mesh triangle number fn + 2 * type1_count + 2 * type2_count;
		//result mesh vertex number max vn + 2 * type1_count + 2 * type2_count;
		unsigned result_triangle_number = m_mesh.triangle_number + 2 * type1_count + 2 * type2_count;
		unsigned result_vertex_max_number = m_mesh.vertex_number + 2 * type1_count + 2 * type2_count;

		Mesh* mesh = new Mesh();
		mesh->AllocateVertex(result_vertex_max_number + edges.size());
		memcpy(mesh->vertex_position, m_mesh.vertex_position, 3 * m_mesh.vertex_number * sizeof(float));
		memcpy(mesh->vertex_normal, m_mesh.vertex_normal, 3 * m_mesh.vertex_number * sizeof(float));
		memcpy(mesh->vertex_color, m_mesh.vertex_color, 3 * m_mesh.vertex_number * sizeof(unsigned char));

		mesh->AllocateTriangle(result_triangle_number);
		mesh->vertex_number = m_mesh.vertex_number;

		BuildMesh(*mesh, triangles, edges);
		return mesh;
	}

	void MeshEdgeStripper::BuildMesh(Mesh& mesh, std::vector<unsigned>& strip_triangles, std::vector<unsigned> edges)
	{
		auto f_add_strip_triangles = [&strip_triangles](unsigned index) {
			strip_triangles.push_back(index);
		};

		auto f_mesh_color = [&mesh](unsigned index)->unsigned char*{
			return mesh.vertex_color + 3 * index;
		};

		auto f_mesh_position = [&mesh](unsigned index)->float* {
			return mesh.vertex_position + 3 * index;
		};

		auto f_mesh_normal = [&mesh](unsigned index)->float* {
			return mesh.vertex_normal + 3 * index;
		};

		auto f_mesh_triangle_index = [&mesh](unsigned index)->unsigned* {
			return mesh.triangle_index + 3 * index;
		};

		//Fill new triangles and vertex
		unsigned* current_triangle_index = mesh.triangle_index;
		unsigned current_triangle_count = 0;
		float* current_vertex = f_mesh_position(m_mesh.vertex_number);
		float* current_normal = f_mesh_normal(m_mesh.vertex_number);
		unsigned char* current_color = f_mesh_color(m_mesh.vertex_number);
		unsigned current_vertex_count = m_mesh.vertex_number;

		auto f_add_one_triangle = [&current_triangle_count, &current_triangle_index](unsigned* triangle) {
			*current_triangle_index++ = *triangle++;
			*current_triangle_index++ = *triangle++;
			*current_triangle_index++ = *triangle++;
			++current_triangle_count;
		};

		auto f_add_one_triangle_ex = [&current_triangle_count, &current_triangle_index](unsigned v1, unsigned v2, unsigned v3) {
			*current_triangle_index++ = v1;
			*current_triangle_index++ = v2;
			*current_triangle_index++ = v3;
			++current_triangle_count;
		};

		auto fill_normal = [&mesh, &current_normal](unsigned index) {
			float* vertex_normal = mesh.vertex_normal + 3 * index;
			memcpy(current_normal, vertex_normal, 3 * sizeof(float));
			current_normal += 3;
		};

		auto fill_color = [&mesh, &current_color](unsigned index) {
			unsigned char* vertex_color = mesh.vertex_color + 3 * index;
			memcpy(current_color, vertex_color, 3 * sizeof(unsigned char));
			current_color += 3;
		};

		auto f_new_vertex = [&current_vertex, &current_normal, &current_color, &current_vertex_count, &mesh](unsigned i1, unsigned i2, float* v1, float* v2, float l)->unsigned {
			unsigned index = current_vertex_count++;
			float* vo = current_vertex;

			*(vo) = *(v1) * (1.0f - l) + *(v2) * l;
			*(vo + 1) = *(v1 + 1) * (1.0f - l) + *(v2 + 1) * l;
			*(vo + 2) = *(v1 + 2) * (1.0f - l) + *(v2 + 2) * l;

			float* vertex_normal = mesh.vertex_normal + 3 * i1;
			memcpy(current_normal, vertex_normal, 3 * sizeof(float));
			current_normal += 3;

			unsigned char* vertex_color = mesh.vertex_color + 3 * i1;
			memcpy(current_color, vertex_color, 3 * sizeof(unsigned char));
			current_color += 3;

			current_vertex += 3;
			return index;
		};

		auto fvertex = [](float* v1, float* v2, float* vo){
			*(vo) = (*(v1)+*(v2)) / 2.0f;
			*(vo + 1) = (*(v1 + 1) + *(v2 + 1)) / 2.0f;
			*(vo + 2) = (*(v1 + 2) + *(v2 + 2)) / 2.0f;
		};

		auto distance = [](float* v0, float* v1)
		{
			return sqrtf(
				(v0[0] - v1[0]) * (v0[0] - v1[0]) +
				(v0[1] - v1[1]) * (v0[1] - v1[1]) +
				(v0[2] - v1[2]) * (v0[2] - v1[2]));
		};

		for (size_t i = 0; i < m_triangles_type_indicator.size(); ++i)
		{
			triangle_type_info& info = m_triangles_type_indicator[i];
			unsigned* old_triangle_index = m_mesh.triangle_index + 3 * i;

			if (info.type == 0)
			{
				f_add_one_triangle(old_triangle_index);
			}
			else if (info.type == 3)
			{
				f_add_strip_triangles(current_triangle_count);
				f_add_one_triangle(old_triangle_index);
			}
			else if (info.type == 1)
			{
				unsigned off = 0;
				for (int i = 0; i < 3; ++i)
				{
					if (info.i[i] == 0)
					{
						off = i;
						break;
					}
				}
				unsigned sindex = old_triangle_index[off];
				unsigned rindex = old_triangle_index[(off + 1) % 3];
				unsigned lindex = old_triangle_index[(off + 2) % 3];

				float* s_vertex_position = f_mesh_position(sindex);
				float* r_vertex_position = f_mesh_position(rindex);
				float* l_vertex_position = f_mesh_position(lindex);

				unsigned new_vertex_index1 = f_new_vertex(sindex, rindex, s_vertex_position, r_vertex_position, 0.1f);
				unsigned new_vertex_index2 = f_new_vertex(sindex, lindex, s_vertex_position, l_vertex_position, 0.1f);

				f_add_strip_triangles(current_triangle_count);
				f_add_one_triangle_ex(sindex, new_vertex_index1, new_vertex_index2);
				f_add_one_triangle_ex(new_vertex_index1, rindex, lindex);
				f_add_one_triangle_ex(new_vertex_index1, lindex, sindex);
			}
			else if (info.type == 2)
			{
				unsigned off = 0;
				for (int i = 0; i < 3; ++i)
				{
					if (info.i[i] == -1)
					{
						off = i;
						break;
					}
				}

				unsigned uindex = old_triangle_index[off];
				unsigned lindex = old_triangle_index[(off + 1) % 3];
				unsigned rindex = old_triangle_index[(off + 2) % 3];

				float* u_vertex_position = f_mesh_position(uindex);
				float* r_vertex_position = f_mesh_position(rindex);
				float* l_vertex_position = f_mesh_position(lindex);

				float edgeLength[3] = 
				{
					distance(r_vertex_position, l_vertex_position), 
					distance(l_vertex_position, u_vertex_position),
					distance(u_vertex_position, r_vertex_position)
				};

				float k = (edgeLength[0] + edgeLength[1] + edgeLength[2]) * 0.5f;
				float Area = sqrtf(k * (k - edgeLength[0]) * (k - edgeLength[1]) * (k - edgeLength[2]));
				float triHeight = 2 * Area / edgeLength[0];

				static float LineHeight = 0.3f * edgeLength[1];
				float l = LineHeight / triHeight;
				l = (l > 1.0f) ? 1.0f : l;

				unsigned new_vertex_index1 = f_new_vertex(lindex, uindex, l_vertex_position, u_vertex_position, l);
				unsigned new_vertex_index2 = f_new_vertex(rindex, uindex, r_vertex_position, u_vertex_position, l);

				f_add_strip_triangles(current_triangle_count);
				f_add_one_triangle_ex(new_vertex_index1, lindex, new_vertex_index2);
				f_add_strip_triangles(current_triangle_count);
				f_add_one_triangle_ex(new_vertex_index2, lindex, rindex);
				f_add_one_triangle_ex(new_vertex_index1, new_vertex_index2, uindex);
			}
		}

		// Duplicate vertex for per-face color
		std::map<unsigned, unsigned> oldVertexToNewVertexMap;
		for (auto edgeVertex : edges)
		{
			float* vertexPosition = f_mesh_position(edgeVertex);
			float tempPosition[3] = { 0.0f, 0.0f, 0.0f };
			unsigned newVertexIndex = f_new_vertex(edgeVertex, -1, vertexPosition, tempPosition, 0.0f);
			auto color = f_mesh_color(newVertexIndex);
			color[0] = 255;
			color[1] = 0;
			color[2] = 0;

			oldVertexToNewVertexMap[edgeVertex] = newVertexIndex;
		}

		for (auto triangle : strip_triangles)
		{
			unsigned* vertexIndices = f_mesh_triangle_index(triangle);
			for (int i = 0; i < 3; i++)
			{
				if (oldVertexToNewVertexMap.count(vertexIndices[i]) > 0)
				{
					vertexIndices[i] = oldVertexToNewVertexMap[vertexIndices[i]];
				}
			}
		}

		mesh.vertex_number = current_vertex_count;
	}
}