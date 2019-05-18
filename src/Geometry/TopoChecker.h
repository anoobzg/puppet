#pragma once

#ifndef TOPO_CHECKER
#define TOPO_CHECKER
#include "LauncaGeometryExport.h"
#include <string>

namespace LauncaGeometry
{
	class Mesh;
	class LAUNCA_GEOMETRY_API MeshTopoChecker
	{
	public:
		static void CheckUnconnectVertex(Mesh& mesh);
	};
}
#endif // TOPO_CHECKER