#ifndef MESH_VERTEX_TRAITS
#define MESH_VERTEX_TRAITS
#include "LauncaGeometryExport.h"

namespace LauncaGeometry
{
	class Mesh;
	class LAUNCA_GEOMETRY_API MeshVertexTraits
	{
	public:
		static void GetTriangleVertices(Mesh& mesh, float* x, float* y, float* z);
		static void GetVertexCoord(Mesh& mesh, unsigned index, float& x, float& y, float& z);
		static void GetVertices(Mesh& mesh, float* x, float* y, float* z, float* nx, float* ny, float* nz, bool skip_repaired);
	};
}
#endif // MESH_VERTEX_TRAITS