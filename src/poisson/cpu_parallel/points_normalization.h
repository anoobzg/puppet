#pragma once
#include "data.h"

void normalization_points(const std::vector<trimesh::vec3>& input_points, const trimesh::box& bounding_box,
	std::vector<trimesh::vec3>& output_points);