#include "ScanFile2Mesh.h"
#include "Mesh.h"

#include <fstream>
namespace LauncaGeometry
{

bool ScanFile2Mesh::Count(const wchar_t* file, unsigned& count)
{
	std::ifstream stream;
	stream.open(file, std::ios::in | std::ios::binary);
	if(!stream.is_open())
	{
		stream.close();
		return false;
	}

	count = 0;
	unsigned frame_size = 0;
	stream.read((char*)&frame_size, sizeof(unsigned));
	if(frame_size <= 0 || frame_size >= 100000)
	{
		stream.close();
		return false;
	}

	for(unsigned i = 0; i < frame_size; ++i)
	{
		unsigned size = 0;
		stream.read((char*)&size, sizeof(unsigned));
		for(unsigned j = 0; j < size; ++j)
		{
			float p[3];
			float n[3];
			unsigned char c[3];
			stream.read((char*)&p[0], 3 * sizeof(float));
			stream.read((char*)&n[0], 3 * sizeof(float));
			stream.read((char*)&c[0], 3 * sizeof(unsigned char));
		}

		count += size;
	}

	stream.close();
	if(count > 0)
		return true;
	return false;
}

Mesh* ScanFile2Mesh::Load(const wchar_t* file)
{
	unsigned count = 0;
	bool result = Count(file, count);
	if(!result)
		return 0;

	return _Load(file, count);
}

Mesh* ScanFile2Mesh::_Load(const wchar_t* file, unsigned count)
{
	std::ifstream stream;
	stream.open(file, std::ios::in | std::ios::binary);
	if(!stream.is_open())
	{
		stream.close();
		return 0;
	}

	unsigned frame_size = 0;
	stream.read((char*)&frame_size, sizeof(unsigned));
	if(frame_size <= 0 || frame_size >= 10000)
	{
		stream.close();
		return 0;
	}

	Mesh* mesh = new Mesh();
	mesh->AllocateVertex(count);

	float* tvertex = mesh->vertex_position;
	float* tnormal = mesh->vertex_normal;
	unsigned char* tcolor = mesh->vertex_color;
	for(unsigned i = 0; i < frame_size; ++i)
	{
		unsigned size = 0;
		stream.read((char*)&size, sizeof(unsigned));
		for(unsigned j = 0; j < size; ++j)
		{
			stream.read((char*)tvertex, 3 * sizeof(float));
			stream.read((char*)tnormal, 3 * sizeof(float));
			stream.read((char*)tcolor, 3 * sizeof(unsigned char));
			tvertex += 3;
			tnormal += 3;
			tcolor += 3;
		}
	}

	stream.close();
	return mesh;
}

}