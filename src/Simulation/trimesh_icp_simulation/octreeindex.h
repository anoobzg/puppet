#pragma once

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