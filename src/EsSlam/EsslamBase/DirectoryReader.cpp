#include "DirectoryReader.h"
#include <boost\format.hpp>
#include "TriMesh.h"
#include "DFrame.h"

namespace esslam
{
	DirectoryReader::DirectoryReader()
		:m_current_index(0)
	{

	}

	DirectoryReader::~DirectoryReader()
	{

	}

	void DirectoryReader::SetParameters(const std::string& directory, const std::string& pattern)
	{
		m_directory = directory;
		m_pattern = pattern;
	}

	void DirectoryReader::Reset()
	{
		m_current_index = 0;
	}

	bool DirectoryReader::Load(DFrame& frame)
	{
		std::string name = m_directory + "//" + boost::str(boost::format(m_pattern.c_str()) % m_current_index);
		++m_current_index;
		std::auto_ptr<trimesh::TriMesh> mesh(trimesh::TriMesh::read(name));

		if (mesh.get())
		{
			int size = (int)mesh->vertices.size();
			frame.data.width = mesh->grid_width;
			frame.data.height = mesh->grid_height;
			frame.data.num_effective = size;
			memcpy(&frame.data.grid[0], &mesh->grid[0], mesh->grid_width * mesh->grid_height * sizeof(int));
			memcpy(&frame.data.points[0], &mesh->vertices[0], size * 3 * sizeof(float));
			memcpy(&frame.data.normals[0], &mesh->normals[0], size * 3 * sizeof(float));
			//frame.data.colors.resize(frame.data.num_effective, trimesh::vec3(1.0f, 1.0f, 1.0f));
			return true;
		}
		
		return false;
	}
}