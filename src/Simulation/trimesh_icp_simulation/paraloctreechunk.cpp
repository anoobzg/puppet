#include "paraloctreechunk.h"

ParalOctreeChunk::ParalOctreeChunk()
	:m_current(0)
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

ParalOctreeChunk::~ParalOctreeChunk()
{

}

void ParalOctreeChunk::QuickInsert(int depth_max, float chunk_resolution_inv, float chunk_resolution, const trimesh::vec3 dmin,
	float cell_resolution_inv,
	std::vector<int*>& cell_indices, const std::vector<trimesh::vec3>& points)
{
	if (m_points_index.size() == 0) return;

	if (m_indxes.size() == 0)
		m_indxes.resize(100000);

	int key_max = 1 << depth_max;
	size_t psize = m_points_index.size();
	for (size_t i = 0; i < psize; ++i)
	{
		const trimesh::vec3& p = points.at(m_points_index[i]);
		trimesh::vec3 np = (p - dmin) * chunk_resolution_inv;
		int x = (int)std::floorf(np.x);
		int y = (int)std::floorf(np.y);
		int z = (int)std::floorf(np.z);
		trimesh::vec3 offset = p - trimesh::vec3((float)x, (float)y, (float)z) * chunk_resolution - dmin;

		IndexKey key;
		trimesh::vec3 noffset = offset * cell_resolution_inv;
		key.x = (int)std::floorf(noffset.x);
		key.y = (int)std::floorf(noffset.y);
		key.z = (int)std::floorf(noffset.z);

		if (key.x < 0 || key.x >= key_max ||
			key.y < 0 || key.y >= key_max ||
			key.z < 0 || key.z >= key_max)
		{
			continue;
		}

		//if (m_current + psize * depth_max >= m_indxes.size())
		//	m_indxes.resize(m_indxes.size() + psize * depth_max);
		short ix = key.x % 2;
		short iy = key.y % 2;
		short iz = key.z % 2;

		int& index = m_children[ix * 4 + iy * 2 + iz];
		if (index < 0) index = m_current++;

		OctreeIndex& octree_index = m_indxes.at(index);
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
			if (iindex < 0) iindex = m_current++;

			current_oc = &m_indxes.at(iindex);


			key.x /= 2;
			key.y /= 2;
			key.z /= 2;
			++depth;
		}

		cell_indices[m_points_index[i]] = &current_oc->m_children[key.x * 4 + key.y * 2 + key.z];
	}
	
	m_points_index.clear();
}

void ParalOctreeChunk::PreInsert(int pos)
{
	m_points_index.push_back(pos);
}