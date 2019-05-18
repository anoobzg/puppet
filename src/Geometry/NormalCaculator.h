#ifndef NORMAL_CACULATOR
#define NORMAL_CACULATOR
#include "LauncaGeometryExport.h"

namespace LauncaGeometry
{
	class Mesh;
	class LAUNCA_GEOMETRY_API NormalCaculator
	{
	public:
		static void CaculateNormalFromTopo(Mesh& mesh);
		static void CaculateFaceNormal(Mesh& mesh, unsigned index, float& nx, float& ny, float& nz);
	};
}
#endif // NORMAL_CACULATOR