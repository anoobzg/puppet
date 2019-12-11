#include "interface.h"

void build_unique_nodes(const std::vector<index_key>& sorted_keys, unsigned max_depth, std::vector<core_node>& nodes)
{
	unsigned size = (unsigned)sorted_keys.size();
	nodes.reserve(size);

	for (unsigned i = 0; i < size; ++i)
	{
		if (i == 0)
		{
			const index_key& ik = sorted_keys[i];
			core_node n;
			n.k = ik.k;
			n.start_index = i;
			n.point_number = 1;

			nodes.push_back(n);
		}
		else
		{
			const index_key& ik1 = sorted_keys[i - 1];
			const index_key& ik2 = sorted_keys[i];
			if (ik2.k != ik1.k)
			{
				core_node n;
				n.k = ik2.k;
				n.start_index = i;
				n.point_number = 1;

				nodes.push_back(n);
			}
			else
			{
				core_node& n = nodes.back();
				++n.point_number;
			}
		}
	}
}