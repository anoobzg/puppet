#include "checker.h"
#include <assert.h>
#include <iostream>

bool check_points_number(const std::vector<core_node>& nodes, unsigned total)
{
	unsigned t = 0;
	for (size_t i = 0; i < nodes.size(); ++i)
	{
		const core_node& n = nodes[i];
		if (n.point_number > 0)
		{
			if (t != n.start_index)
				std::cout << "error." << std::endl;
			t += n.point_number;
		}
	}
	return t == total;
}

bool check_relation(const std::vector<node>& all_nodes, const std::vector<depth_info>& infos)
{
	bool result = true;
	for (unsigned d = 1; d < infos.size(); ++d)
	{
		const depth_info& info = infos[d];
		for (unsigned i = info.start_index; i < info.node_number; ++i)
		{
			const node& n = all_nodes[i];
			int pindex = n.n.parent;
			if (pindex >= 0)
			{
				const node& pn = all_nodes[pindex];
				if (pn.n.k != parent_key(n.n.k, d))
				{
					std::cout << "parent error." << std::endl;
					result = false;
					break;
				}
			}

			for (unsigned j = 0; j < 8; ++j)
			{
				int cindex = n.n.children[j];
				if (cindex >= 0)
				{
					const node& cn = all_nodes[cindex];
					if (n.n.k != parent_key(cn.n.k, d + 1))
					{
						std::cout << "children error." << std::endl;
						result = false;
						break;
					}
				}
			}
		}

		if (!result)
			break;
	}
	return result;
}

bool check_neighbors(const node& n, unsigned index)
{
	bool result = n.neighbors[13] == index;
	if (!result) std::cout << "neighbor error" << std::endl;
	return result;
}

bool check_relation_ex(const std::vector<node>& all_nodes, const std::vector<depth_info>& infos)
{
	bool result = true;
	for (unsigned d = 1; d < infos.size(); ++d)
	{
		const depth_info& info = infos[d];
		for (unsigned i = info.start_index; i < info.node_number; ++i)
		{
			unsigned index = i + info.start_index;
			const node& pn = all_nodes[index];

			for (unsigned j = 0; j < 8; ++j)
			{
				if (pn.n.children[j] >= 0)
				{
					const node& cn = all_nodes[pn.n.children[j]];
					if (cn.n.parent != index)
					{
						std::cout << "error relation in " << index << "node." << std::endl;
						result = false;
						break;
					}
				}
			}

			if (!result)
				break;
		}
		if (!result)
			break;
	}

	return result;
}