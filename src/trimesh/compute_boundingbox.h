#pragma once
#include "Box.h"
#include "Vec.h"
#include "Xform.h"
#include <vector>

namespace trimesh
{
	void ComputeBoundingbox(const std::vector<trimesh::point>& points,
		trimesh::box3& box);

	void TransformBoundingbox(const trimesh::box3& box, const trimesh::xform& xf,
		float& xmin, float& xmax, float& ymin, float& ymax, float& zmin, float& zmax);

	void TransformBoundingbox(const std::vector<trimesh::point>& points, const trimesh::xform& xf,
		float& xmin, float& xmax, float& ymin, float& ymax, float& zmin, float& zmax);
}