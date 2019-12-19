#include "validator.h"

void Validator::Validate(std::vector<trimesh::vec3>& positions,
	std::vector<trimesh::vec3>& normals)
{
	size_t size = positions.size();
	size_t valid_index = 0;
	for (size_t i = 0; i < size; ++i)
	{
		const trimesh::vec3& p = positions.at(i);
		const trimesh::vec3& n = normals.at(i);

		if (::_isnanf(p.x) || ::_isnanf(p.y) || ::_isnanf(p.z)
			|| ::_isnanf(n.x) || ::_isnanf(n.y) || ::_isnanf(n.z)
			|| (trimesh::len(n) == 0.0f))
			continue;

		positions.at(valid_index) = p;
		normals.at(valid_index) = trimesh::normalized(n);
		++valid_index;
	}
}