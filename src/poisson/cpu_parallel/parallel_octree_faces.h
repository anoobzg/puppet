#pragma once
#include "data.h"

class ParallelOctreeNodes;
class ParalledOctreeFaces
{
public:
	ParalledOctreeFaces();
	~ParalledOctreeFaces();

	void BuildFromNodes(const ParallelOctreeNodes& nodes);

private:
	std::vector<octree_face> m_faces;
};