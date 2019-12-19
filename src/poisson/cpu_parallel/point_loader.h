#pragma once
#include "data.h"

class PointLoader
{
public:
	static void LoadFromFile(const char* file, std::vector<trimesh::vec3>& positions, 
		std::vector<trimesh::vec3>& normals);

	static void LoadFromPlyFile(const char* file, std::vector<trimesh::vec3>& positions,
		std::vector<trimesh::vec3>& normals);
};