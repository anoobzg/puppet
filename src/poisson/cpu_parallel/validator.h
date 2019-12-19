#pragma once
#include "data.h"

class Validator
{
public:
	static void Validate(std::vector<trimesh::vec3>& positions,
		std::vector<trimesh::vec3>& normals);
};
