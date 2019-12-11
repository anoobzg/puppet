#include "interface.h"
#include "key.h"

void build_points_key(const std::vector<point>& points, const vec3& recon_max, const vec3& recon_min, unsigned depth, std::vector<index_key>& ikeys)
{
	unsigned size = (unsigned)points.size();
	ikeys.resize(size);

	for (unsigned i = 0; i < size; ++i)
	{
		index_key& ik = ikeys[i];
		ik.index = i;

		unsigned k = 0;
		const vec3& position = points[i].position;

		vec3 t_min = recon_min;
		vec3 t_max = recon_max;
		for (unsigned d = 1; d <= depth; ++d)
		{
			vec3 center;
			center.x = (t_min.x + t_max.x) / 2.0f;
			center.y = (t_min.y + t_max.y) / 2.0f;
			center.z = (t_min.z + t_max.z) / 2.0f;

			//x
			if (position.x < center.x)
			{
				key_write_left(k, d, 0);
				t_max.x = center.x;
			}
			else
			{
				key_write_right(k, d, 0);
				t_min.x = center.x;
			}

			//y
			if (position.y < center.y)
			{
				key_write_left(k, d, 1);
				t_max.y = center.y;
			}
			else
			{
				key_write_right(k, d, 1);
				t_min.y = center.y;
			}

			//z
			if (position.z < center.z)
			{
				key_write_left(k, d, 2);
				t_max.z = center.z;
			}
			else
			{
				key_write_right(k, d, 2);
				t_min.z = center.z;
			}
		}

		ik.k = k;
	}
}

void build_map_points(std::vector<point>& points, const vec3& recon_max, const vec3& recon_min)
{
	unsigned size = (unsigned)points.size();

	float dx = recon_max.x - recon_min.x;
	float dy = recon_max.y - recon_min.y;
	float dz = recon_max.z - recon_min.z;
	for (unsigned i = 0; i < size; ++i)
	{
		point& p = points[i];
		p.position.x = (p.position.x - recon_min.x) / dx;
		p.position.y = (p.position.y - recon_min.y) / dy;
		p.position.z = (p.position.z - recon_min.z) / dz;
	}
}