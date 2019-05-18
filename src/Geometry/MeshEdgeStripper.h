#pragma once
#include "LauncaGeometryExport.h"
#include <vector>

namespace LauncaGeometry
{
	struct triangle_type_info
	{
		unsigned type; // 0, 1, 2, 3
		int i[3];

		triangle_type_info()
		{
			i[0] = i[1] = i[2] = -1;
			type = 0;
		}
	};

	class Mesh;
	class LAUNCA_GEOMETRY_API MeshEdgeStripper
	{
	public:
		MeshEdgeStripper(Mesh& mesh);
		~MeshEdgeStripper();

		Mesh* Do(/*in edge vertex index*/std::vector<unsigned>& edges, /*out triangle index*/std::vector<unsigned>& triangles);
	protected:
		bool CheckValid(std::vector<unsigned>& edges);
		bool BuildUpData(std::vector<unsigned>& edges);
		void BuildMesh(Mesh& out, std::vector<unsigned>& strip_triangles, std::vector<unsigned> edges);
	private:
		Mesh& m_mesh;

		std::vector<triangle_type_info> m_triangles_type_indicator;
		std::vector<int> m_vertex_mapping;
	};
}