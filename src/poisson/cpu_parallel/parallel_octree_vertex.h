#pragma once
#include "data.h"

class ParallelOctreeNodes;
class ParalledOctreeVertex
{
public:
	ParalledOctreeVertex();
	~ParalledOctreeVertex();

	void BuildFromNodes(const ParallelOctreeNodes& nodes);

private:
	std::vector<octree_vertex> m_vertexes;
};