#include "octree.h"
#include <assert.h>
#include <ppl.h>
#include "timestamp.h"

#define USE_PPL 0
Octree::Octree(int cell_depth, float cell_resolution)
	:m_initialized(false), m_cell_depth(cell_depth), m_cell_resolution(cell_resolution)
	, m_chunk_depth(7), m_chunk_size(1<<7)
	, m_chunk_resolution(6.4f), m_len(6.4f * 128.0f)
	, m_chunk_resolution_inv(1.0f / m_chunk_resolution), m_chunk_size2(m_chunk_size * m_chunk_size)
	, m_current_index(0), m_current_point_index(0), m_cell_resolution_inv(1.0f / m_cell_resolution)
{
	assert(6.4f == (1 << m_cell_depth) * m_cell_resolution);
	m_chunks.resize(m_chunk_size2 * m_chunk_size);
	m_indexes.resize(10000000);
	//m_points.reserve(8000000);
	//m_normals.reserve(8000000);
	m_trimesh.vertices.reserve(8000000);
	m_trimesh.normals.reserve(8000000);
	m_quick_index.reserve(500000);
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

void Octree::Insert(const std::vector<trimesh::vec3>& points,
	const std::vector<trimesh::vec3>& normals)
{
	assert(m_initialized);
	size_t size = points.size();
	for (size_t i = 0; i < size; ++i)
	{
		const trimesh::vec3& p = points.at(i);
		const trimesh::vec3& n = normals.at(i);
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

		trimesh::vec3 offset = p - trimesh::vec3((float)x, (float)y, (float)z) * m_chunk_resolution - m_min;

		IndexKey key;
		trimesh::vec3 noffset = offset * m_cell_resolution_inv;
		key.x = (int)std::floorf(noffset.x);
		key.y = (int)std::floorf(noffset.y);
		key.z = (int)std::floorf(noffset.z);

		OctreeChunk& chunk = m_chunks.at(index);
		chunk.SetValid();

		int point_index = chunk.AddPoint(m_cell_depth, m_current_index, m_current_point_index, key, m_indexes);
		if (point_index < 0)
			continue;

		if (point_index < m_current_point_index) //exist average
		{
			trimesh::vec3& pp = m_trimesh.vertices.at(point_index);
			pp += p;
			pp /= 2.0f;
		}

		if (point_index == m_current_point_index) //new 
		{
			m_trimesh.vertices.push_back(p);
			m_trimesh.normals.push_back(n);
			++m_current_point_index;
		}
	}
}

void Octree::Insert(const std::vector<trimesh::vec3>& points,
	const std::vector<trimesh::vec3>& normals, const trimesh::xform& xf,
	std::vector<int>& indexes)
{
	assert(m_initialized);
	size_t size = points.size();
	trimesh::xform nxf = trimesh::norm_xf(xf);
	for (size_t i = 0; i < size; ++i)
	{
		trimesh::vec3 p = xf * points.at(i);
		trimesh::vec3 n = nxf * normals.at(i);

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

		trimesh::vec3 offset = p - trimesh::vec3((float)x, (float)y, (float)z) * m_chunk_resolution - m_min;

		IndexKey key;
		trimesh::vec3 noffset = offset * m_cell_resolution_inv;
		key.x = (int)std::floorf(noffset.x);
		key.y = (int)std::floorf(noffset.y);
		key.z = (int)std::floorf(noffset.z);

		OctreeChunk& chunk = m_chunks.at(index);
		int point_index = chunk.AddPoint(m_cell_depth, m_current_index, m_current_point_index, key, m_indexes);

		indexes.at(i) = point_index;
		if (point_index < 0)
			continue;

		if (point_index < m_current_point_index) //exist average
		{
			trimesh::vec3& pp = m_trimesh.vertices.at(point_index);
			pp += p;
			pp /= 2.0f;
		}
		if (point_index == m_current_point_index) //new 
		{
			//trimesh::vec3 nnoffset = offset * m_cell_resolution_inv;
			//int kx = (int)std::floorf(nnoffset.x);
			//int ky = (int)std::floorf(nnoffset.y);
			//int kz = (int)std::floorf(nnoffset.z);

			//trimesh::vec3 pp = trimesh::vec3((float)x, (float)y, (float)z) * m_chunk_resolution 
			//	+ trimesh::vec3((float)kx, (float)ky, (float)kz) * m_cell_resolution + m_min;
			//m_points.push_back(pp);
			m_trimesh.vertices.push_back(p);
			m_trimesh.normals.push_back(n);
			++m_current_point_index;
		}
	}
}

void Octree::QuickInsert(const std::vector<trimesh::vec3>& points, const std::vector<trimesh::vec3>& normals)
{
	assert(m_initialized);
	size_t size = points.size();
	if (size == 0) return;

	//trimesh::timestamp t0 = trimesh::now();

	m_quick_index.resize(size);
	int key_max = 1 << m_cell_depth;

#if USE_PPL
	Concurrency::parallel_for<size_t>(0, size, [this, &key_max, &points](size_t i) {
#else
	for (size_t i = 0; i < size; ++i) {
#endif
		const trimesh::vec3& p = points.at(i);
		QuickIndex& quick_index = m_quick_index.at(i);
		std::vector<char>& cell_indexes = quick_index.cell_index;
		cell_indexes.resize(m_cell_depth + 1);
		quick_index.chunk_index = -1;

		trimesh::vec3 np = (p - m_min) * m_chunk_resolution_inv;
		int x = (int)std::floorf(np.x);
		int y = (int)std::floorf(np.y);
		int z = (int)std::floorf(np.z);

		if (x < 0 || x >= m_chunk_size)
#if USE_PPL
			return;
#else
			continue;
#endif
		if (y < 0 || y >= m_chunk_size)
#if USE_PPL
			return;
#else
			continue;
#endif
		if (z < 0 || z >= m_chunk_size)
#if USE_PPL
			return;
#else
			continue;
#endif

		quick_index.chunk_index = x + y * m_chunk_size + z * m_chunk_size2;

		trimesh::vec3 offset = p - trimesh::vec3((float)x, (float)y, (float)z) * m_chunk_resolution - m_min;

		IndexKey key;
		trimesh::vec3 noffset = offset * m_cell_resolution_inv;
		key.x = (int)std::floorf(noffset.x);
		key.y = (int)std::floorf(noffset.y);
		key.z = (int)std::floorf(noffset.z);

		if (key.x < 0 || key.x >= key_max ||
			key.y < 0 || key.y >= key_max ||
			key.z < 0 || key.z >= key_max)
		{
			quick_index.chunk_index = -1;
#if USE_PPL
			return;
#else
			continue;
#endif
		}

		short ix = key.x % 2;
		short iy = key.y % 2;
		short iz = key.z % 2;

		cell_indexes.at(0) = ix * 4 + iy * 2 + iz;

		key.x /= 2;
		key.y /= 2;
		key.z /= 2;

		int depth = 0;
		while (depth < m_cell_depth - 2)
		{
			short iix = key.x % 2;
			short iiy = key.y % 2;
			short iiz = key.z % 2;

			cell_indexes.at(depth + 1) = iix * 4 + iiy * 2 + iiz;

			key.x /= 2;
			key.y /= 2;
			key.z /= 2;
			++depth;
		}

		cell_indexes.at(depth + 2) = key.x * 4 + key.y * 2 + key.z;
#if USE_PPL
	});
#else
	}
#endif
		

	//trimesh::timestamp t1 = trimesh::now();

	for (size_t i = 0; i < size; ++i)
	{
		QuickIndex& quick_index = m_quick_index.at(i);
		if (quick_index.chunk_index < 0) continue;
		OctreeChunk& chunk = m_chunks.at(quick_index.chunk_index);
		chunk.SetValid();

		int point_index = chunk.AddPoint(m_cell_depth, m_current_index, m_current_point_index, m_indexes, quick_index.cell_index);
		assert(point_index >= 0);

		const trimesh::vec3& p = points.at(i);
		const trimesh::vec3& n = points.at(i);
		if (point_index < m_current_point_index) //exist average
		{
			trimesh::vec3& pp = m_trimesh.vertices.at(point_index);
			pp += p;
			pp /= 2.0f;
		}
		
		if (point_index == m_current_point_index) //new 
		{
			m_trimesh.vertices.push_back(p);
			m_trimesh.normals.push_back(n);
			++m_current_point_index;
		}
	}

	//trimesh::timestamp t2 = trimesh::now();
	//std::cout << (t1 - t0) << "  " << (t2 - t1) << std::endl;
}