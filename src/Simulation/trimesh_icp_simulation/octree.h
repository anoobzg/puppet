#pragma once
#include "Vec.h"
#include "octreechunk.h"
#include <vector>
#include "octreeindex.h"
#include "Xform.h"

class Octree
{
public:
	Octree(int cell_depth = 5, float cell_resolution = 0.2f);
	~Octree();

	void Initialize(const trimesh::vec3& center);
	void Insert(const std::vector<trimesh::vec3>& points, const std::vector<trimesh::vec3>& normals);
	void Insert(const std::vector<trimesh::vec3>& points,
		const std::vector<trimesh::vec3>& normals,
		const trimesh::xform& xf, std::vector<int>& indexes);
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

	std::vector<OctreeChunk> m_chunks;

	std::vector<OctreeIndex> m_indexes;
	std::vector<trimesh::vec3> m_points;
	std::vector<trimesh::vec3> m_normals;
	int m_current_index;
	int m_current_point_index;
};