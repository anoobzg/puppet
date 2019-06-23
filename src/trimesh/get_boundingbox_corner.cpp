#include "get_boundingbox_corner.h"

namespace trimesh
{
	void GetBoundingboxCorner(const trimesh::box3& box, std::vector<trimesh::point>& points)
	{
		const trimesh::point& pmax = box.max;
		const trimesh::point& pmin = box.min;

		points.resize(8);
		points.at(0) = pmin;
		points.at(1) = trimesh::point(pmax.x, pmin.y, pmin.z);
		points.at(2) = (trimesh::point(pmax.x, pmax.y, pmin.z));
		points.at(3) = pmax;
		points.at(4) = (trimesh::point(pmin.x, pmax.y, pmax.z));
		points.at(5) = (trimesh::point(pmax.x, pmin.y, pmax.z));
		points.at(6) = (trimesh::point(pmin.x, pmin.y, pmax.z));
		points.at(7) = (trimesh::point(pmin.x, pmax.y, pmin.z));
	}
}