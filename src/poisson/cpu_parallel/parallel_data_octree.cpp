#include "parallel_data_octree.h"
#include "boundingbox_calculator.h"

ParallelDataOctree::ParallelDataOctree()
	:m_depth(10), m_box_extend(0.01f)
{

}

ParallelDataOctree::~ParallelDataOctree()
{

}

void ParallelDataOctree::SetParameters(int depth, float box_extend)
{
	m_depth = depth;
	m_box_extend = box_extend;
}

void ParallelDataOctree::BuildFromPoints(const std::vector<trimesh::vec3>& positions,
	const std::vector<trimesh::vec3>& normals)
{
	BuildBoundingBox(positions);

	m_nodes.Build(positions, normals, m_build_box, m_depth);
	m_vertexes.BuildFromNodes(m_nodes);
	m_edges.BuildFromNodes(m_nodes);
	m_faces.BuildFromNodes(m_nodes);
}

void ParallelDataOctree::BuildBoundingBox(const std::vector<trimesh::vec3>& positions)
{
	m_real_box = calculate_boundingbox(positions);

	trimesh::vec3 bsize = m_real_box.size();
	float len = std::fmaxf(bsize.x, std::fmaxf(bsize.y, bsize.z));
	trimesh::vec3 bcenter = m_real_box.center();

	len *= (1.0f + m_box_extend);
	trimesh::vec3 half_size = trimesh::vec3(len, len, len);
	m_build_box.valid = true;
	m_build_box.max = bcenter + half_size;
	m_build_box.min = bcenter - half_size;
}