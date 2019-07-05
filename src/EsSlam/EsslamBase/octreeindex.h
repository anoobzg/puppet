#pragma once
#include <vector>
#include "EsslamBaseExport.h"

namespace esslam
{
	struct IndexKey
	{
		short x;
		short y;
		short z;
	};

	class OctreeIndex
	{
	public:
		OctreeIndex();
		~OctreeIndex();

		int m_children[8];
	};

	struct QuickIndex
	{
		int chunk_index;
		std::vector<char> cell_index;
	};
}