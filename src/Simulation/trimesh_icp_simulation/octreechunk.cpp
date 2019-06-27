#include "octreechunk.h"

OctreeChunk::OctreeChunk()
	:m_valid(false)
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

OctreeChunk::~OctreeChunk()
{

}

void OctreeChunk::SetMin(const trimesh::vec3& bmin)
{
	m_bmin = bmin;
}

int OctreeChunk::AddPoint(int depth_max, int& current, int current_point_index, IndexKey& key,
	std::vector<OctreeIndex>& indexes)
{
	int key_max = 1 << depth_max;
	if (key.x < 0 || key.x >= key_max ||
		key.y < 0 || key.y >= key_max ||
		key.z < 0 || key.z >= key_max)
		return -1;

	short ix = key.x % 2;
	short iy = key.y % 2;
	short iz = key.z % 2;

	int& index = m_children[ix * 4 + iy * 2 + iz];
	if (index < 0) index = current++;

	OctreeIndex& octree_index = indexes.at(index);
	key.x /= 2;
	key.y /= 2;
	key.z /= 2;

	OctreeIndex* current_oc = &octree_index;
	int depth = 0;
	while (depth < depth_max - 2)
	{
		short iix = key.x % 2;
		short iiy = key.y % 2;
		short iiz = key.z % 2;

		int& iindex = current_oc->m_children[iix * 4 + iiy * 2 + iiz];
		if (iindex < 0) iindex = current++;

		current_oc = &indexes.at(iindex);
		

		key.x /= 2;
		key.y /= 2;
		key.z /= 2;
		++depth;
	}

	int& point_index = current_oc->m_children[key.x * 4 + key.y * 2 + key.z];
	if (point_index < 0)
		point_index = current_point_index;
	return point_index;
}