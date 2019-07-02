#pragma once

#include "octreeindex.h"
#include <vector>
#include "Vec.h"

class ParalOctreeChunk
{
public:
	ParalOctreeChunk();
	~ParalOctreeChunk();

	void PreInsert(int pos);
	void QuickInsert(int depth_max, float chunk_resolution_inv, float chunk_resolution, const trimesh::vec3 dmin,
		float cell_resolution_inv,
		std::vector<int*>& cell_indices, const std::vector<trimesh::vec3>& points);
private:
	int m_children[8];
	int m_current;
	std::vector<OctreeIndex> m_indxes;

	std::vector<int> m_points_index;
};