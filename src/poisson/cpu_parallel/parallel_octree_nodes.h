#pragma once
#include "data.h"

class ParallelOctreeNodes
{
	friend class ParalledOctreeVertex;
	friend class ParalledOctreeEdges;
	friend class ParalledOctreeFaces;
public:
	ParallelOctreeNodes();
	~ParallelOctreeNodes();

	void Build(const std::vector<trimesh::vec3>& positions, const std::vector<trimesh::vec3>& normals,
		const trimesh::box& bounding_box, int depth);

	void Clear();
protected:
	std::vector<trimesh::vec3> m_sorted_positions;
	std::vector<trimesh::vec3> m_sorted_normals;

	std::vector<octree_node> m_nodes;
};