#include "MeshScaler.h"
#include "Mesh.h"

namespace LauncaGeometry
{
	void MeshScaler::Scale(float x, float y, float z, Mesh& mesh)
	{
		for(unsigned i = 0; i < mesh.vertex_number; ++i)
		{
			float* p = mesh.vertex_position + 3 * i;
			*p = *p * x; *(p+1) = *(p+1) * x; *(p+2) = *(p+2) * x;
		}
	}
}