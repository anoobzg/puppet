#include "interface.h"
#include <iostream>

void build_octree_parameters(const std::vector<point>& points, vec3& recon_max, vec3& recon_min, unsigned& depth, float& grid_size)
{
	vec3 max; max.x = FLT_MIN; max.y = FLT_MIN; max.z = FLT_MIN;
	vec3 min; min.x = FLT_MAX; min.y = FLT_MAX; min.z = FLT_MAX;
	for (size_t i = 0; i < points.size(); ++i)
	{
		const point& p = points[i];
		if (min.x > p.position.x) min.x = p.position.x;
		if (min.y > p.position.y) min.y = p.position.y;
		if (min.z > p.position.z) min.z = p.position.z;
		if (max.x < p.position.x) max.x = p.position.x;
		if (max.y < p.position.y) max.y = p.position.y;
		if (max.z < p.position.z) max.z = p.position.z;
	}

	float dx = max.x - min.x;
	float dy = max.y - min.y;
	float dz = max.z - min.z;
	float len = std::fmaxf(dx, std::fmaxf(dy, dz));
	float stable_size = 80.0f;

	//if (len > 1024.0f * stable_size)
	//{
	//	recon_max = max;
	//	recon_min = min;
	//	depth = 10;
	//	grid_size = len / (powf(2.0f, 10.0f));
	//}
	//else if (len > 512.0f * stable_size)
	//{
	//	recon_max = min;
	//	recon_min = min;
	//	recon_max.x += 1024.0f * stable_size;
	//	recon_max.y += 1024.0f * stable_size;
	//	recon_max.z += 1024.0f * stable_size;
	//	depth = 10;
	//	grid_size = stable_size;
	//}
	//else if (len > 256.0f * stable_size)
	//{
	//	recon_max = min;
	//	recon_min = min;
	//	recon_max.x += 512.0f * stable_size;
	//	recon_max.y += 512.0f * stable_size;
	//	recon_max.z += 512.0f * stable_size;
	//	depth = 9;
	//	grid_size = stable_size;
	//}
	//else
	//{
	//	recon_min = max;
	//	recon_min = min;
	//	recon_max.x += 256.0f * stable_size;
	//	recon_max.y += 256.0f * stable_size;
	//	recon_max.z += 256.0f * stable_size;
	//	depth = 8;
	//	grid_size = stable_size;
	//}
	vec3 center;
	center.x = (min.x + max.x) / 2.0f;
	center.y = (min.y + max.y) / 2.0f;
	center.z = (min.z + max.z) / 2.0f;

	recon_min.x = center.x - len / 2.0f;
	recon_min.y = center.y - len / 2.0f;
	recon_min.z = center.z - len / 2.0f;
	recon_max.x = center.x + len / 2.0f;
	recon_max.y = center.y + len / 2.0f;
	recon_max.z = center.z + len / 2.0f;
	depth = 10;
	grid_size = 10.0f;
}