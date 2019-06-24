#pragma once
#include "TriMesh.h"


class Data
{
public:
	Data();
	~Data();

	void Load(const std::string& directory);

protected:
	void Release();
private:
	std::vector<trimesh::TriMesh*> m_meshes;
};
