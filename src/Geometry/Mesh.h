#ifndef GEOMETRY_MESH
#define GEOMETRY_MESH
#include "LauncaGeometryExport.h"

namespace LauncaGeometry
{
	class LAUNCA_GEOMETRY_API Mesh
	{
	public:
		Mesh();
		~Mesh();

		void AllocateVertex(unsigned vertex_number);
		void AllocateTriangle(unsigned triangle_number);

		unsigned vertex_number;
		unsigned triangle_number;
		float*   vertex_position;
		float*   vertex_normal;
		unsigned char* vertex_color;
		unsigned* triangle_index;
		char* triangle_char;

		int m_repair_vertex_append_index;
		int m_repair_triangle_append_index;

	private:
		void ReleaseVertex();
		void ReleaseTriangles();
	};
}
#endif // GEOMETRY_MESH