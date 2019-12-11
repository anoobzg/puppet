#include "interface.h"

void build_sorted_points(const std::vector<point>& points, std::vector<index_key>& sorted_keys, std::vector<point>& sorted_points)
{
	sorted_points.resize(points.size());
	for (size_t i = 0; i < points.size(); ++i)
	{
		index_key& ik = sorted_keys[i];
		sorted_points[i] = points[ik.index];
	}
}