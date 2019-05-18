#pragma once

#include "LauncaGeometryExport.h"
#include "Mesh.h"

namespace LauncaGeometry
{
	class LAUNCA_GEOMETRY_API TopoMesh
	{
	public:
		TopoMesh(Mesh& mesh);
		~TopoMesh();

	private:
		Mesh& m_mesh;
	};
}