#ifndef MESH_ERASOR
#define MESH_ERASOR
#include "LauncaGeometryExport.h"

namespace LauncaGeometry
{
	class Mesh;
	class LAUNCA_GEOMETRY_API MeshErasor
	{
	public:
		static Mesh* Erase(Mesh& mesh, float* flag, bool& is_empty);
	};
}

#endif // MESH_ERASOR