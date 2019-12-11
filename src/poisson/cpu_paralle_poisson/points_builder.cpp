#include "interface.h"

void build_raw_points(PointCloud& cloud, std::vector<point>& points)
{
	unsigned n = cloud.vertex_number;
	points.resize(n);
	for (unsigned i = 0; i < n; ++i)
	{
		float* p = cloud.vertex_position + 3 * i;
		float* n = cloud.vertex_normal + 3 * i;
		point& pp = points[i];
		pp.position.x = *p++; pp.position.y = *p++; pp.position.z = *p++;
		pp.normal.x = *n++; pp.normal.y = *n++; pp.normal.z = *n++;
	}
}