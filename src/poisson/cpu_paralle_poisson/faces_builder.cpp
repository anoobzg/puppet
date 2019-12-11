#include "interface.h"
#include  "key.h"

void build_node_faces(const std::vector<node>& nodes, const depth_info& info, std::vector<node_face>& nfaces)
{
	nfaces.resize(info.node_number);
	for (unsigned i = 0; i < info.node_number; ++i)
	{
		const node& n = nodes[info.start_index + i];
		node_face& nf = nfaces[i];
		nf.nindex = info.start_index + i;
		for (unsigned j = 0; j < 6; ++j)
		{
			key k = -1;
			for (unsigned f = 0; f < 2; ++f)
			{
				unsigned nindex = face_neighbors_table[j][f];
				if (n.neighbors[nindex] >= 0)
				{
					nf.nodes[j][f] = n.neighbors[nindex];
					key kk = nodes[n.neighbors[nindex]].n.k;
					if (kk < k) k = kk;
				}
			}

			nf.fkey[j] = k;
		}
	}
}

void build_full_faces(const std::vector<std::vector<node_face>>& nfaces, std::vector<face>& faces)
{
	unsigned index = 0;
	for (size_t i = 0; i < nfaces.size(); ++i)
	{
		const std::vector<node_face>& dface = nfaces[i];
		for (size_t j = 0; j < dface.size(); ++j)
		{
			const node_face& nf = dface[j];
			for (unsigned f = 0; f < 6; ++f)
			{
				if (nf.fkey[f] != 1)
				{
					face& fa = faces[index];
					memcpy(fa.nodes, nf.nodes[f], 2 * sizeof(int));
					++index;
				}
			}
		}
	}
}