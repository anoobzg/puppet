#include "compute_boundingbox.h"
#include <assert.h>

namespace trimesh
{

	void ComputeBoundingbox(const std::vector<trimesh::point>& points,
		trimesh::box3& box)
	{
		trimesh::point& box_min = box.min;
		trimesh::point& box_max = box.max;

		if (points.empty()) {
			box_min.x = box_min.y = box_min.z = 0.0f;
			box_max.x = box_max.y = box_max.z = 0.0f;
			return;
		}

		// Find bounding box of pts
		box_min.x = box_max.x = points[0][0];
		box_min.y = box_max.y = points[0][1];
		box_min.z = box_max.z = points[0][2];
		size_t npts = points.size();
		for (size_t i = 1; i < npts; i++)
		{
			const trimesh::point& p = points.at(i);
			if (p.x < box_min.x) box_min.x = p.x;
			else if (p.x > box_max.x) box_max.x = p.x;
			if (p.y < box_min.y) box_min.y = p.y;
			else if (p.y > box_max.y) box_max.y = p.y;
			if (p.z < box_min.z) box_min.z = p.z;
			else if (p.z > box_max.z) box_max.z = p.z;
		}

		box.valid = true;
	}

	void TransformBoundingbox(const trimesh::box3& box, const trimesh::xform& xf,
		float& xmin, float& xmax, float& ymin, float& ymax, float& zmin, float& zmax)
	{

	}

	void TransformBoundingbox(const std::vector<trimesh::point>& points, const trimesh::xform& xf,
		float& xmin, float& xmax, float& ymin, float& ymax, float& zmin, float& zmax)
	{
		xmin = ymin = zmin = FLT_MAX;
		xmax = ymax = zmax = -FLT_MAX;

		size_t npts = points.size();
		assert(npts > 0);
		for (size_t i = 0; i < npts; i++)
		{
			trimesh::point p = xf * points[i];

			if (p.x < xmin) xmin = p.x;
			else if (p.x > xmax) xmax = p.x;
			if (p.y < ymin) ymin = p.y;
			else if (p.y > ymax) ymax = p.y;
			if (p.z < zmin) zmin = p.z;
			else if (p.z > zmax) zmax = p.z;
		}
	}

}