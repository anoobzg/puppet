#pragma once
#include "data.h"

namespace trimesh
{
	class TriMesh;
}

class RawPointCloud
{
public:
	RawPointCloud();
	~RawPointCloud();

	void BuildFromTrimesh(const trimesh::TriMesh& mesh);
public:
	std::vector<trimesh::vec3> m_positions;
	std::vector<trimesh::vec3> m_normals;
	size_t m_number;
	trimesh::box m_boundingbox;
};
