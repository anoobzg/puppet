#pragma once

#include "Vec.h"
#include "octreechunk.h"
#include <vector>
#include "paraloctreechunk.h"
#include "octreeindex.h"
#include "Xform.h"

class ParalOctree
{
public:
	ParalOctree(int cell_depth = 6, float cell_resolution = 0.2f);
	~ParalOctree();

	void Initialize(const trimesh::vec3& center);

	void QuickInsert(const std::vector<trimesh::vec3>& points, const std::vector<trimesh::vec3>& normals);
protected:

public:
	bool m_initialized;
	const int m_cell_depth;
	const float m_cell_resolution;
	const float m_cell_resolution_inv;
	const float m_len;
	const int m_chunk_size;
	const int m_chunk_depth;
	const float m_chunk_resolution;
	const float m_chunk_resolution_inv;
	const int m_chunk_size2;

	trimesh::vec3 m_center;
	trimesh::vec3 m_min;

	std::vector<trimesh::vec3> m_points;
	std::vector<trimesh::vec3> m_normals;
	int m_current_point_index;

	std::vector<ParalOctreeChunk> m_chunks;
};