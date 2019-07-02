#include "paraloctree.h"
#include <assert.h>
#include <ppl.h>
#include "timestamp.h"

#define USE_PPL 1
ParalOctree::ParalOctree(int cell_depth, float cell_resolution)
	:m_initialized(false), m_cell_depth(cell_depth), m_cell_resolution(cell_resolution)
	, m_chunk_depth(6), m_chunk_size(1 << 6)
	, m_chunk_resolution(12.8f), m_len(12.8f * 64.0f)
	, m_chunk_resolution_inv(1.0f / m_chunk_resolution), m_chunk_size2(m_chunk_size * m_chunk_size)
	, m_current_point_index(0), m_cell_resolution_inv(1.0f / m_cell_resolution)
{
	assert(12.8f == (1 << m_cell_depth) * m_cell_resolution);
	m_chunks.resize(m_chunk_size2 * m_chunk_size);
	m_points.reserve(8000000);
	m_normals.reserve(8000000);
}

ParalOctree::~ParalOctree()
{

}

void ParalOctree::Initialize(const trimesh::vec3& center)
{
	m_center = center;
	m_min = center - trimesh::vec3(1.0f, 1.0f, 1.0f) * m_len / 2.0f;

	m_initialized = true;
}

void ParalOctree::QuickInsert(const std::vector<trimesh::vec3>& points, const std::vector<trimesh::vec3>& normals)
{
	assert(m_initialized);
	size_t size = points.size();
	if (size == 0) return;

	std::vector<bool> dirty(m_chunk_size2 * m_chunk_size, false);
	int key_max = 1 << m_cell_depth;
	for (size_t i = 0; i < size; ++i)
	{
		const trimesh::vec3& p = points.at(i);

		trimesh::vec3 np = (p - m_min) * m_chunk_resolution_inv;
		int x = (int)std::floorf(np.x);
		int y = (int)std::floorf(np.y);
		int z = (int)std::floorf(np.z);

		if (x < 0 || x >= m_chunk_size)
			continue;

		if (y < 0 || y >= m_chunk_size)
			continue;

		if (z < 0 || z >= m_chunk_size)
			continue;

		int index = x + y * m_chunk_size + z * m_chunk_size2;
		m_chunks[index].PreInsert((int)i);
		dirty[index] = true;
	}

	std::vector<int*> cell_indices(size, NULL);
	std::vector<ParalOctreeChunk*> dirty_chunks;
	dirty_chunks.reserve(size);
	size_t all_chunk_size = m_chunks.size();
	for (size_t i = 0; i < all_chunk_size; ++i)
		if (dirty[i])
			dirty_chunks.push_back(&m_chunks[i]);

	size_t all_chunk = dirty_chunks.size();
	Concurrency::parallel_for<size_t>(0, all_chunk, [this, &dirty_chunks, &cell_indices, &points](size_t i)
	{
		dirty_chunks[i]->QuickInsert(m_cell_depth, m_chunk_resolution_inv, m_chunk_resolution,
			m_min, m_cell_resolution_inv, cell_indices, points);
	});

	for (size_t i = 0; i < size; ++i)
	{
		int* pointer = cell_indices[i];
		if (pointer == NULL) continue;

		int& point_index = *pointer;
		const trimesh::vec3& p = points.at(i);
		const trimesh::vec3& n = points.at(i);
		if (point_index >= 0) //exist average
		{
			trimesh::vec3& pp = m_points.at(point_index);
			pp += p;
			pp /= 2.0f;
		}else if(point_index < 0) //new 
		{
			m_points.push_back(p);
			m_normals.push_back(n);
			point_index = m_current_point_index++;
		}
	}
}