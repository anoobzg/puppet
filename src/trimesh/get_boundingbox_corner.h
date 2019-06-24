#pragma once
#include "Vec.h"
#include "Box.h"
#include <vector>

namespace trimesh
{
	void GetBoundingboxCorner(const trimesh::box3& box, std::vector<trimesh::point>& points);
}