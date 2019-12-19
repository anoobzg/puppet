#pragma once
#include "data.h"

class ParallelOctreeNodes;
class ParalledOctreeEdges
{
public:
	ParalledOctreeEdges();
	~ParalledOctreeEdges();

	void BuildFromNodes(const ParallelOctreeNodes& nodes);

private:
	std::vector<octree_edge> m_edges;
};