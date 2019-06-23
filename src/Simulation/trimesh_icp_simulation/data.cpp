#include "data.h"

Data::Data()
{

}

Data::~Data()
{
	Release();
}

void Data::Load(const std::string& directory)
{
	Release();

	char name[32];
	for (int i = 0; i < 100; ++i)
	{
		sprintf(name, "%d.ply", i);
		std::string file = directory + "\\" + name;
		trimesh::TriMesh* mesh = trimesh::TriMesh::read(file);
		if (!mesh) break;

		m_meshes.push_back(mesh);
		std::cout << "Load " << file.c_str() << std::endl;
	}
}

void Data::Release()
{
	for (size_t i = 0; i < m_meshes.size(); ++i)
	{
		delete m_meshes[i];
		m_meshes[i] = NULL;
	}
	m_meshes.clear();
}