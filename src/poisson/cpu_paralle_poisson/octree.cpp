#include "octree.h"
#include "interface.h"
#include <algorithm>
#include "key.h"

#include <fstream>
#include <iostream>

#include "checker.h"

Octree::Octree(unsigned depth)
	:m_depth(depth)
{
	m_depth_info.resize(m_depth + 1);
}

Octree::~Octree()
{

}

void Octree::BuildSortedPoints(const std::vector<point>& points, const vec3& max, const vec3& min, unsigned depth,
	std::vector<index_key>& sorted_keys, std::vector<point>& sorted_points)
{
	build_points_key(points, max, min, depth, sorted_keys);
	std::sort(sorted_keys.begin(), sorted_keys.end(), index_key_compare());

	build_sorted_points(points, sorted_keys, sorted_points);

	build_map_points(sorted_points, max, min);
}

void Octree::BuildUniqueNodes(const std::vector<index_key>& sorted_keys, const std::vector<point>& sorted_points, unsigned max_depth,
	std::vector<core_node>& nodes)
{
	build_unique_nodes(sorted_keys, max_depth, nodes);
}

void Octree::BuildAllDepthNodes(std::vector<core_node>& nodes, std::vector<std::vector<core_node>>& all_depth_nodes, unsigned max_depth)
{
	//for (size_t i = 0; i < all_depth_nodes.size(); ++i)
	//{
	//	std::vector<core_node>& full_nodes = all_depth_nodes[i];
	//	if (full_nodes.size() > 1000 || full_nodes.capacity() > 1000)
	//		std::cout << "error." << std::endl;
	//}
	std::vector<core_node> max_nodes;
	expand_nodes(nodes, max_nodes, max_depth, all_depth_nodes[max_depth]);
	
	//for (size_t i = 0; i < all_depth_nodes.size(); ++i)
	//{
	//	std::vector<core_node>& full_nodes = all_depth_nodes[i];
	//	if (full_nodes.size() > 1000 || full_nodes.capacity() > 1000)
	//		std::cout << "error." << std::endl;
	//}
	//bool result = check_points_number(all_depth_nodes[max_depth], (unsigned)m_sorted_points.size());
	for (unsigned i = max_depth; i > 0; --i)
	{
		BuildRecursive(all_depth_nodes[i], i - 1, all_depth_nodes[i - 1]);
		//result = check_points_number(all_depth_nodes[i], (unsigned)m_sorted_points.size());

		for (size_t j = 0; j < all_depth_nodes.size(); ++j)
		{
			std::vector<core_node>& full_nodes = all_depth_nodes[j];
			//if (full_nodes.size() > 1000 || full_nodes.capacity() > 1000)
			//	std::cout << "error." << std::endl;
		}
	}

	unsigned total_nodes_number = 0;
	for (unsigned i = 0; i <= m_depth; ++i)
	{
		depth_info& dinfo = m_depth_info[i];
		dinfo.start_index = total_nodes_number;
		dinfo.node_number = (unsigned)all_depth_nodes[i].size();
		total_nodes_number += dinfo.node_number;
	}
	
	m_nodes.resize(total_nodes_number);
	for (unsigned i = 0; i <= m_depth; ++i)
	{
		const depth_info& di = m_depth_info[i];
		const std::vector<core_node>& dnodes = all_depth_nodes[i];
		for (size_t j = 0; j < dnodes.size(); ++j)
			m_nodes[di.start_index + j].n = dnodes[j];
	}
	
	for (unsigned d = 0; d < m_depth; ++d)
	{
		const std::vector<core_node>& parent_nodes = all_depth_nodes[d];
		const std::vector<core_node>& children_nodes = all_depth_nodes[d + 1];
		const depth_info& pdi = m_depth_info[d];
		const depth_info& cdi = m_depth_info[d + 1];
		BuildDepthRelationRecursive(parent_nodes, children_nodes, pdi, cdi, m_nodes);
	}

	//bool result = check_relation(m_nodes, m_depth_info);
	//bool result = check_relation_ex(m_nodes, m_depth_info);
}

void Octree::BuildRecursive(std::vector<core_node>& children_full_nodes, unsigned depth, std::vector<core_node>& parent_full_nodes)
{
	std::vector<core_node> parent_dense_nodes;
	//if (children_full_nodes.capacity() > 100 || parent_full_nodes.capacity() > 100)
	//	std::cout << "error." << std::endl;

	build_next_depth_nodes(children_full_nodes, depth + 1, parent_dense_nodes);

	//if (children_full_nodes.capacity() > 100 || parent_full_nodes.capacity() > 100)
	//	std::cout << "error." << std::endl;

	expand_nodes(parent_dense_nodes, children_full_nodes, depth, parent_full_nodes);

	//if (children_full_nodes.capacity() > 100 || parent_full_nodes.capacity() > 100)
	//	std::cout << "error." << std::endl;
}

void Octree::BuildDepthRelationRecursive(const std::vector<core_node>& parent_nodes, const std::vector<core_node>& child_nodes,
	const depth_info& parent_depth_info, const depth_info& child_depth_info, std::vector<node>& nodes)
{
	fill_childrens(parent_nodes, parent_depth_info, child_depth_info, nodes);
	fill_parents(child_nodes, parent_depth_info, child_depth_info, nodes);
}

void Octree::BuildNeighbors(std::vector<node>& nodes, std::vector<depth_info>& depth_infos)
{
	node& root_node = nodes[0];
	root_node.neighbors[13] = 0;

	for (unsigned i = 1; i <= m_depth; ++i)
	{
		const depth_info& df = depth_infos[i];
		for (unsigned j = 0; j < df.node_number; ++j)
		{
			unsigned index = df.start_index + j;
			node& n = nodes[index];
			unsigned ckey = depth_key(n.n.k, i);
			node& p = nodes[n.n.parent];
			for (unsigned k = 0; k < 27; ++k)
			{
				unsigned pnindex = parent_lu_table[ckey][k];
				if (p.neighbors[pnindex] != -1)
				{
					node& h = nodes[p.neighbors[pnindex]];
					n.neighbors[k] = h.n.children[child_lu_table[ckey][k]];
				}
				else
					n.neighbors[k] = -1;
			}

			//check_neighbors(n, index);
		}
	}
}

void Octree::BuildVertices()
{
	std::vector<std::vector<node_vertex>> all_node_vertices(m_depth + 1);
	for (size_t i = 0; i <= m_depth; ++i)
	{
		const depth_info& df = m_depth_info[i];
		std::vector<node_vertex>& nvertices = all_node_vertices[i];
		build_node_vertices(m_nodes, df, nvertices);
	}

	unsigned total_vertices_number = 0;
	m_vertices_depth_info.resize(m_depth + 1);
	for (size_t i = 0; i <= m_depth; ++i)
	{
		depth_info& vdf = m_vertices_depth_info[i];
		vdf.start_index = total_vertices_number;
		unsigned num = 0;
		std::vector<node_vertex>& nvertices = all_node_vertices[i];
		for (size_t j = 0; j < nvertices.size(); ++j)
		{
			node_vertex& nv = nvertices[j];
			node& n = m_nodes[nv.nindex];
			n.vertices.start_index = total_vertices_number;

			for (unsigned v = 0; v < 8; ++v)
			{
				if (nv.vkey[v] == n.n.k)
				{
					++num;
					++total_vertices_number;
				}
				else
				{
					nv.vkey[v] = 1;//dumplicated
				}
			}
			n.vertices.node_number = total_vertices_number - n.vertices.start_index;
		}

		vdf.node_number = num;
	}

	m_vertices.resize(total_vertices_number);
	build_full_vertices(all_node_vertices, m_vertices);
}

void Octree::BuildEdges()
{
	std::vector<std::vector<node_edge>> all_node_edges(m_depth + 1);
	for (size_t i = 0; i <= m_depth; ++i)
	{
		const depth_info& df = m_depth_info[i];
		std::vector<node_edge>& edges = all_node_edges[i];
		build_node_edges(m_nodes, df, edges);
	}

	unsigned total_edges_number = 0;
	m_edges_depth_info.resize(m_depth + 1);
	for (size_t i = 0; i <= m_depth; ++i)
	{
		depth_info& ddf = m_edges_depth_info[i];
		ddf.start_index = total_edges_number;
		unsigned num = 0;
		std::vector<node_edge>& nedges = all_node_edges[i];
		for (size_t j = 0; j < nedges.size(); ++j)
		{
			node_edge& ne = nedges[j];
			node& n = m_nodes[ne.nindex];
			n.edges.start_index = total_edges_number;

			for (unsigned e = 0; e < 12; ++e)
			{
				if (ne.ekey[e] == n.n.k)
				{
					++num;
					++total_edges_number;
				}
				else
				{
					ne.ekey[e] = 1;//dumplicated
				}
			}
			n.edges.node_number = total_edges_number - n.edges.start_index;
		}

		ddf.node_number = num;
	}

	m_edges.resize(total_edges_number);
	build_full_edges(all_node_edges, m_edges);
}

void Octree::BuildFaces()
{
	std::vector<std::vector<node_face>> all_node_faces(m_depth + 1);
	for (size_t i = 0; i <= m_depth; ++i)
	{
		const depth_info& df = m_depth_info[i];
		std::vector<node_face>& faces = all_node_faces[i];
		build_node_faces(m_nodes, df, faces);
	}

	unsigned total_faces_number = 0;
	m_faces_depth_info.resize(m_depth + 1);
	for (size_t i = 0; i <= m_depth; ++i)
	{
		depth_info& fdf = m_faces_depth_info[i];
		fdf.start_index = total_faces_number;
		unsigned num = 0;
		std::vector<node_face>& faces = all_node_faces[i];
		for (size_t j = 0; j < faces.size(); ++j)
		{
			node_face& nf = faces[j];
			node& n = m_nodes[nf.nindex];
			n.edges.start_index = total_faces_number;

			for (unsigned f = 0; f < 6; ++f)
			{
				if (nf.fkey[f] == n.n.k)
				{
					++num;
					++total_faces_number;
				}
				else
				{
					nf.fkey[f] = 1;//dumplicated
				}
			}
			n.edges.node_number = total_faces_number - n.edges.start_index;
		}

		fdf.node_number = num;
	}

	m_faces.resize(total_faces_number);
	build_full_faces(all_node_faces, m_faces);
}

void Octree::Build(const std::vector<point>& points, const vec3& max, const vec3& min)
{
	std::vector<index_key> ikeys;
	BuildSortedPoints(points, max, min, m_depth, ikeys, m_sorted_points);

	m_point_phi.resize(m_sorted_points.size(), 0.0f);

	std::vector<core_node> max_depth_dense_nodes;
	BuildUniqueNodes(ikeys, m_sorted_points, m_depth, max_depth_dense_nodes);

	std::vector<std::vector<core_node>> all_depth_nodes;
	all_depth_nodes.resize(m_depth + 1);

	BuildAllDepthNodes(max_depth_dense_nodes, all_depth_nodes, m_depth);

	ikeys.clear();
	max_depth_dense_nodes.clear();
	all_depth_nodes.clear();
	m_phi.resize(m_nodes.size(), 0.0f);
	
	BuildNeighbors(m_nodes, m_depth_info);
	
	BuildVertices();
	BuildEdges();
	BuildFaces();
}





