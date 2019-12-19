#include "points_normalization.h"

void normalization_points(const std::vector<trimesh::vec3>& input_points, const trimesh::box& bounding_box,
	std::vector<trimesh::vec3>& output_points)
{
	size_t size = input_points.size();
	trimesh::vec3 blen = bounding_box.size();
	trimesh::vec3 bmin = bounding_box.min;
	for (size_t i = 0; i < size; ++i)
	{
		const trimesh::vec3& p = input_points.at(i);
		trimesh::vec3& normalization_p = output_points.at(i);
		normalization_p.x = (p.x - bmin.x) / blen.x;
		normalization_p.y = (p.y - bmin.y) / blen.y;
		normalization_p.z = (p.z - bmin.z) / blen.z;
	}
}