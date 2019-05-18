#include "TopoChecker.h"
#include "Mesh.h"

#include <iostream>
#include <vector>
namespace LauncaGeometry
{
	void MeshTopoChecker::CheckUnconnectVertex(Mesh& mesh)
	{
		std::vector<bool> connectd(mesh.vertex_number, false);
		for (unsigned i = 0; i < mesh.triangle_number; ++i)
		{
			unsigned* index = mesh.triangle_index + 3 * i;
			connectd[*index++] = true;
			connectd[*index++] = true;
			connectd[*index++] = true;
		}

		for (unsigned i = 0; i < mesh.vertex_number; ++i)
			if (connectd[i] == false)
				std::cout << "Vertex " << i << " Is Unconnected." << std::endl;
	}
}