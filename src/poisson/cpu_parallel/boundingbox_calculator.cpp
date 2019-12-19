#include "boundingbox_calculator.h"

trimesh::box calculate_boundingbox(const std::vector<trimesh::vec3>& positions)
{
	trimesh::box bbox;
	size_t size = positions.size();
	for (size_t i = 0; i < size; ++i)
		bbox += positions.at(i);
	return bbox;
}