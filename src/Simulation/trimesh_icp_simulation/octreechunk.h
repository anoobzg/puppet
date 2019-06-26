#pragma once
#include "Vec.h"

class OctreeChunk
{
public:
	OctreeChunk();
	~OctreeChunk();

	inline bool Valid() { return m_valid; }
	inline void SetValid() { m_valid = true; }
	void SetMin(const trimesh::vec3& bmin);
private:
	bool m_valid;
	int m_children[8];

public:
	trimesh::vec3 m_bmin;
};