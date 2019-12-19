#include "point_loader.h"

#include <PointCloudLoader.h>

#include <pcl/io/auto_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "TriMesh.h"

bool EndWith(const char* name, const char* ext)
{
	int name_length = static_cast<int>(strlen(name));
	int ext_length = static_cast<int>(strlen(ext));

	if (ext_length > name_length) return false;
	const char* name_ext = name + name_length - ext_length;
	return strcmp(name_ext, ext) == 0;
}

void PointLoader::LoadFromFile(const char* file, std::vector<trimesh::vec3>& positions,
	std::vector<trimesh::vec3>& normals)
{
	if (EndWith(file, ".ply"))
		LoadFromPlyFile(file, positions, normals);
}

void PointLoader::LoadFromPlyFile(const char* file, std::vector<trimesh::vec3>& positions,
	std::vector<trimesh::vec3>& normals)
{
	std::unique_ptr<trimesh::TriMesh> mesh(trimesh::TriMesh::read(file));
	if (mesh.get())
	{
		positions.swap(mesh->vertices);
		normals.swap(mesh->normals);
	}
}