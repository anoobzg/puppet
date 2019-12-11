#include "interface.h"
#include "key.h"

#include <stdio.h>
void build_node_vertices(const std::vector<node>& nodes, const depth_info& info, std::vector<node_vertex>& nvertices)
{
	nvertices.resize(info.node_number);
	for (unsigned i = 0; i < info.node_number; ++i)
	{
		const node& n = nodes[info.start_index + i];
		node_vertex& nv = nvertices[i];
		nv.nindex = info.start_index + i;
		for (unsigned j = 0; j < 8; ++j)
		{
			key k = -1;
			for (unsigned v = 0; v < 8; ++v)
			{
				unsigned nindex = vertex_neighbors_table[j][v];
				if (n.neighbors[nindex] >= 0)
				{
					nv.nodes[j][v] = n.neighbors[nindex];
					key kk = nodes[n.neighbors[nindex]].n.k;
					if (kk < k) k = kk;
				}
			}

			nv.vkey[j] = k;
		}
	}
}

void build_full_vertices(const std::vector<std::vector<node_vertex>>& nvertices, std::vector<vertex>& vertices)
{
	unsigned index = 0;
	for (size_t i = 0; i < nvertices.size(); ++i)
	{
		const std::vector<node_vertex>& dvertex = nvertices[i];
		for (size_t j = 0; j < dvertex.size(); ++j)
		{
			const node_vertex& nv = dvertex[j];
			for (unsigned v = 0; v < 8; ++v)
			{
				if (nv.vkey[v] != 1)
				{
					vertex& ver = vertices[index];
					memcpy(ver.nodes, nv.nodes[v], 8 * sizeof(int));
					++index;
				}
			}
		}
	}
}