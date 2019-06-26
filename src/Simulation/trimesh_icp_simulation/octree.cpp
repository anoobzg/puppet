#include "octree.h"
#include <assert.h>

Octree::Octree()
	:m_initialized(false), m_depth(13), m_cell_resolution(0.1f)
	, m_len(m_cell_resolution * (1 << m_depth)), m_chunk_depth(7), m_chunk_size(1<<7)
	, m_chunk_resolution(m_cell_resolution * (1<<6))
	, m_chunk_resolution_inv(1.0f / m_chunk_resolution), m_chunk_size2(m_chunk_size * m_chunk_size)
{
	m_chunks.resize(m_chunk_size2 * m_chunk_size);
}

Octree::~Octree()
{

}

void Octree::Initialize(const trimesh::vec3& center)
{
	m_center = center;
	m_min = center - trimesh::vec3(1.0f, 1.0f, 1.0f) * m_len / 2.0f;

	for (int i = 0; i < m_chunk_size; ++i)
	{
		for (int j = 0; j < m_chunk_size; ++j)
		{
			for (int k = 0; k < m_chunk_size; ++k)
			{
				m_chunks.at(i + j * m_chunk_size + k * m_chunk_size2).SetMin(m_min + trimesh::vec3((float)i, (float)j, (float)k) * m_chunk_resolution);
			}
		}
	}
	m_initialized = true;
}

void Octree::Insert(const std::vector<trimesh::vec3>& points)
{
	assert(m_initialized);
	size_t size = points.size();
	for (size_t i = 0; i < size; ++i)
	{
		const trimesh::vec3& p = points.at(i);
		trimesh::vec3 np = (p - m_min) * m_chunk_resolution_inv;
		int x = (int)std::floorf(np.x);
		int y = (int)std::floorf(np.y);
		int z = (int)std::floorf(np.z);

		if (x < 0) x = 0;
		if (x >= m_chunk_size) x = m_chunk_size - 1;
		if (y < 0) y = 0;
		if (y >= m_chunk_size) y = m_chunk_size - 1;
		if (z < 0) z = 0;
		if (z >= m_chunk_size) z = m_chunk_size - 1;

		int index = x + y * m_chunk_size + z * m_chunk_size2;
		m_chunks.at(index).SetValid();
	}
}