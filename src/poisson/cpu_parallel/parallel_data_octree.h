#pragma once
#include "parallel_octree_nodes.h"
#include "parallel_octree_vertex.h"
#include "paralled_octree_edges.h"
#include "parallel_octree_faces.h"

class ParallelDataOctree
{
public:
	ParallelDataOctree();
	~ParallelDataOctree();

	void SetParameters(int depth, float box_extend);
	void BuildFromPoints(const std::vector<trimesh::vec3>& positions,
		const std::vector<trimesh::vec3>& normals);
protected:
	void BuildBoundingBox(const std::vector<trimesh::vec3>& positions);
private:
	int m_depth;
	float m_box_extend;

	trimesh::box m_build_box;
	trimesh::box m_real_box;

	ParallelOctreeNodes m_nodes;
	ParalledOctreeVertex m_vertexes;
	ParalledOctreeEdges m_edges;
	ParalledOctreeFaces m_faces;
};
