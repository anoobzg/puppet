#include "parallel_octree_nodes.h"
#include "points_normalization.h"
#include "index_key_caculator.h"
#include "unique_nodes_builder.h"
#include "node_slicer_builder.h"
#include "node_neighbors.h"
#include "octree_vertex_builder.h"

ParallelOctreeNodes::ParallelOctreeNodes()
{

}

ParallelOctreeNodes::~ParallelOctreeNodes()
{

}

void ParallelOctreeNodes::Build(const std::vector<trimesh::vec3>& positions, const std::vector<trimesh::vec3>& normals,
	const trimesh::box& bounding_box, int depth)
{
	Clear();

	size_t size = positions.size();
	if (size == 0) return;

	std::vector<trimesh::vec3> normalization_positions(size);
	normalization_points(positions, bounding_box, normalization_positions);

	std::vector<index_key> index_keys(size);
	calculate_index_key(normalization_positions, depth, index_keys);

	std::sort(index_keys.begin(), index_keys.end(), index_key_compare());
	m_sorted_normals.resize(size);
	m_sorted_positions.resize(size);

	sort_by_index_key(normalization_positions, normals, index_keys, m_sorted_positions, m_sorted_normals);

	std::vector<unique_node> unique_nodes;
	build_unique_nodes(index_keys, depth, unique_nodes);
	build_all_nodes(unique_nodes, m_nodes, depth);

	fill_nodes_neighbors(m_nodes);
}

void ParallelOctreeNodes::Clear()
{

}