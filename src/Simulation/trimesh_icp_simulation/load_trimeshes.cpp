#include "load_trimeshes.h"
#include "compute_boundingbox.h"
#include <boost\format.hpp>

void load_trimeshes(const std::string& directory, std::vector<trimesh::TriMesh*>& meshes)
{
	int index = 0;
	while (true)
	{
		std::string file = directory + "//" +
			boost::str(boost::format("%d.ply") % index);
		trimesh::TriMesh* mesh = trimesh::TriMesh::read(file);
		if (mesh)
		{
			trimesh::ComputeBoundingbox(mesh->vertices, mesh->bbox);
			meshes.push_back(mesh);
		}
		else break;

		++index;
	}
}