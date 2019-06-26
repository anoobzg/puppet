#pragma once
#include "Vec.h"
#include "octreechunk.h"
#include <vector>

class Octree
{
public:
	Octree();
	~Octree();

	void Initialize(const trimesh::vec3& center);
	void Insert(const std::vector<trimesh::vec3>& points);

public:
	bool m_initialized;
	const int m_depth;
	const float m_cell_resolution;
	const float m_len;
	const int m_chunk_size;
	const int m_chunk_depth;
	const float m_chunk_resolution;
	const float m_chunk_resolution_inv;
	const int m_chunk_size2;

	trimesh::vec3 m_center;
	trimesh::vec3 m_min;

	std::vector<OctreeChunk> m_chunks;
};