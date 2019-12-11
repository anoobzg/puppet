#include "interface.h"
#include "key.h"

#include <iostream>
void expand_nodes(const std::vector<core_node>& dense_nodes, std::vector<core_node>& child_full_nodes, unsigned depth, std::vector<core_node>& full_nodes)
{
	//if (full_nodes.size() > 1000 || full_nodes.capacity() > 1000)
	//	std::cout << "error." << std::endl;
	//if (child_full_nodes.size() > 1000 || child_full_nodes.capacity() > 1000)
	//	std::cout << "error." << std::endl;

	unsigned size = (unsigned)dense_nodes.size();
	unsigned full_num = 8;
	if (depth == 0) full_num = 1;
	for (unsigned i = 1; i < size; ++i)
	{
		const core_node& n1 = dense_nodes[i - 1];
		const core_node& n2 = dense_nodes[i];
		if (!same_parent(n1.k, n2.k, depth))
			full_num += 8;
	}

	full_nodes.resize(full_num); 
	std::vector<unsigned> delta(size, 0);
	unsigned index = 0;
	for (unsigned i = 0; i < size; ++i)
	{
		const core_node& n = dense_nodes[i];
		if (i == 0)
		{
			unsigned d = depth_key(n.k, depth);
			//if (index + d >= full_num)
			//	std::cout << "error." << std::endl;

			full_nodes[index + d] = n;
			delta[i] = index + d;
	
			if (size != 1)
			{
				unsigned pk = parent_key(n.k, depth);
				for (unsigned j = 0; j < 8; ++j)
				{
					//if (index + j >= full_num)
					//	std::cout << "error." << std::endl;

					full_nodes[index + j].k = child_key(pk, depth, j);
				}
			}
		}
		else
		{
			unsigned d = depth_key(n.k, depth);
			const core_node& pn = dense_nodes[i - 1];
			if (same_parent(pn.k, n.k, depth))
			{
				//if (index + d >= full_num)
				//	std::cout << "error." << std::endl;

				core_node& nn = full_nodes[index + d];
				nn = n;
			}
			else
			{
				index += 8;

				//if (index + d >= full_num)
				//	std::cout << "error." << std::endl;

				full_nodes[index + d] = n;
	
				unsigned pk = parent_key(n.k, depth);
				for (unsigned j = 0; j < 8; ++j)
				{
					//if (index + d >= full_num)
					//	std::cout << "error." << std::endl;

					unsigned ck = child_key(pk, depth, j);
					full_nodes[index + j].k = ck;
				}
			}
	
			delta[i] = index + d;
		}
	}

	//if (full_nodes.size() > 1000 || full_nodes.capacity() > 1000)
	//	std::cout << "error." << std::endl;
	//if (child_full_nodes.size() > 1000 || child_full_nodes.capacity() > 1000)
	//	std::cout << "error." << std::endl;

	if (child_full_nodes.size() > 0)
	{
		for (unsigned i = 0; i < size; ++i)
		{
			unsigned index = delta[i];
			for (unsigned j = 0; j < 8; ++j)
			{
				unsigned cindex = 8 * i + j;
				child_full_nodes[cindex].parent = index;
			}
		}
	}
	else
	{
		for (size_t i = 0; i < full_nodes.size(); ++i)
		{
			core_node& n = full_nodes[i];
			n.n_number = 1;
			n.n_start_index = (unsigned)i;
		}
	}

	//if (full_nodes.size() > 1000 || full_nodes.capacity() > 1000)
	//	std::cout << "error." << std::endl;
}