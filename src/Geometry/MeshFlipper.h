#ifndef MESH_FLIPPER
#define MESH_FLIPPER
#include "LauncaGeometryExport.h"

namespace LauncaGeometry
{
	class Mesh;
	class LAUNCA_GEOMETRY_API MeshFlipper
	{
	public:
		static void Flip(Mesh& mesh);
	};
}

#endif // MESH_FLIPPER