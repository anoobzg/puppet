#include "MeshFlipper.h"
#include "Mesh.h"

namespace LauncaGeometry
{
	void MeshFlipper::Flip(Mesh& mesh)
	{
		float* normal = mesh.vertex_normal;
		for(unsigned i = 0; i < mesh.vertex_number; ++i)
		{
			*normal++ = -*normal;
			*normal++ = -*normal;
			*normal++ = -*normal;
		}

		unsigned* triangle = mesh.triangle_index;
		for(unsigned i = 0; i < mesh.triangle_number; ++i)
		{
			triangle++;
			unsigned t = *triangle;
			*triangle++ = *(triangle+1);
			*triangle++ = t;
		}
	}
}