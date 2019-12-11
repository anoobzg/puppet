#include "interface.h"

void fill_childrens(const std::vector<core_node>& parents_nodes, const depth_info& parent_info, const depth_info& children_info, std::vector<node>& nodes)
{
	unsigned start_index = parent_info.start_index;
	unsigned cstart_index = children_info.start_index;
	for (unsigned i = 0; i < parent_info.node_number; ++i)
	{
		node& n = nodes[i + start_index];

		for (unsigned j = 0; j < 8; ++j)
			if (n.n.children[j] >= 0)
				n.n.children[j] += cstart_index;
	}
}

void fill_parents(const std::vector<core_node>& children_nodes, const depth_info& parent_info, const depth_info& children_info, std::vector<node>& nodes)
{
	unsigned start_index = children_info.start_index;
	unsigned pstart_index = parent_info.start_index;
	for (unsigned i = 0; i < children_info.node_number; ++i)
	{
		node& n = nodes[i + start_index];

		n.n.parent += pstart_index;
	}
}