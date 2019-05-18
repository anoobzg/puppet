#ifndef REPAIRED_REMOVER
#define REPAIRED_REMOVER
#include "LauncaGeometryExport.h"

namespace LauncaGeometry
{
	class Mesh;
	class LAUNCA_GEOMETRY_API RepairedRemover
	{
	public:
		static Mesh* Remove(Mesh& mesh);
	};
}

#endif // REPAIRED_REMOVER