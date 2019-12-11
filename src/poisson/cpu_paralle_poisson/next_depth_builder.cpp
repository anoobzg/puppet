#include "interface.h"
#include "key.h"

void build_next_depth_nodes(std::vector<core_node>& full_nodes, unsigned depth, std::vector<core_node>& next_dense_nodes)
{
	unsigned size = (unsigned)full_nodes.size();
	unsigned next_size = size / 8;
	next_dense_nodes.resize(next_size);
	for (unsigned i = 0; i < next_size; ++i)
	{
		core_node& pn = next_dense_nodes[i];
		pn.start_index = -1;
		pn.n_start_index = -1;
		for (unsigned j = 0; j < 8; ++j)
		{
			core_node& n = full_nodes[ 8 * i + j];
			pn.children[j] = 8 * i + j;
			n.parent = i;
			if (j == 0)
				pn.k = parent_key(n.k, depth);
			if (n.point_number > 0)
			{
				pn.point_number += n.point_number;
				if (pn.start_index > n.start_index)
					pn.start_index = n.start_index;
			}
			if (n.n_number > 0)
			{
				pn.n_number += n.n_number;
				if (pn.n_start_index > n.n_start_index)
					pn.n_start_index = n.n_start_index;
			}
		}
	}
}