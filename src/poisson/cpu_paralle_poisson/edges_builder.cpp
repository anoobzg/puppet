#include "interface.h"
#include "key.h"

void build_node_edges(const std::vector<node>& nodes, const depth_info& info, std::vector<node_edge>& nedges)
{
	nedges.resize(info.node_number);
	for (unsigned i = 0; i < info.node_number; ++i)
	{
		const node& n = nodes[info.start_index + i];
		node_edge& ne = nedges[i];
		ne.nindex = info.start_index + i;
		for (unsigned j = 0; j < 12; ++j)
		{
			key k = -1;
			for (unsigned e = 0; e < 4; ++e)
			{
				unsigned nindex = edge_neighbors_table[j][e];
				if (n.neighbors[nindex] >= 0)
				{
					ne.nodes[j][e] = n.neighbors[nindex];
					key kk = nodes[n.neighbors[nindex]].n.k;
					if (kk < k) k = kk;
				}
			}

			ne.ekey[j] = k;
		}
	}
}

void build_full_edges(const std::vector<std::vector<node_edge>>& nedges, std::vector<edge>& edges)
{
	unsigned index = 0;
	for (size_t i = 0; i < nedges.size(); ++i)
	{
		const std::vector<node_edge>& dedge = nedges[i];
		for (size_t j = 0; j < dedge.size(); ++j)
		{
			const node_edge& ne = dedge[j];
			for (unsigned e = 0; e < 12; ++e)
			{
				if (ne.ekey[e] != 1)
				{
					edge& edg = edges[index];
					memcpy(edg.nodes, ne.nodes[e], 4 * sizeof(int));
					++index;
				}
			}
		}
	}
}