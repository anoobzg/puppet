#include "OctreeIndex.h"

namespace esslam
{
	OctreeIndex::OctreeIndex()
	{
		m_children[0] = -1;
		m_children[1] = -1;
		m_children[2] = -1;
		m_children[3] = -1;
		m_children[4] = -1;
		m_children[5] = -1;
		m_children[6] = -1;
		m_children[7] = -1;
	}

	OctreeIndex::~OctreeIndex()
	{

	}
}