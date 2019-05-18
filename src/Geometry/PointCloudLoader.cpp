#include "PointCloudLoader.h"
#include <fstream>
#include "PointCloud.h"
#include <map>
#include <vector>

#include <Windows.h>
namespace LauncaGeometry
{

	PointCloudLoader::PointCloudLoader()
	{
	}

	PointCloudLoader::~PointCloudLoader()
	{
	}

	PointCloud* PointCloudLoader::LoadFromFileName(const wchar_t* file_name)
	{
		std::wstring file = file_name;
		size_t pos = std::string::npos;
		if ((pos = file.rfind('.')) == std::wstring::npos)
			return 0;

		std::string extension = std::string(file.begin() + pos, file.end());
		if (!strcmp(extension.c_str(), ".bin"))
			return LoadFromBin(file);
		else if (!strcmp(extension.c_str(), ".npts"))
			return LoadFromNPTS(file);
		return 0;
	}

	PointCloud* PointCloudLoader::LoadFromFileName(const char* file_name)
	{
		std::string file = file_name;
		size_t pos = std::string::npos;
		if ((pos = file.rfind('.')) == std::wstring::npos)
			return 0;

		std::string extension = std::string(file.begin() + pos, file.end());
		if (!strcmp(extension.c_str(), ".bin"))
			return LoadFromBin(file);
		else if (!strcmp(extension.c_str(), ".npts"))
			return LoadFromNPTS(file);

		return 0;
	}

	PointCloud* PointCloudLoader::LoadFromBin(const std::string& mesh_file)
	{
		std::ifstream in;
		in.open(mesh_file, std::ios::in | std::ios::binary);
		if (!in.is_open())
			return 0;

		return LoadFromBin(in);
	}

	PointCloud* PointCloudLoader::LoadFromBin(const std::wstring& mesh_file)
	{
		std::ifstream in;
		in.open(mesh_file, std::ios::in | std::ios::binary);
		if (!in.is_open())
			return 0;

		return LoadFromBin(in);
	}

	PointCloud* PointCloudLoader::LoadFromBin(std::ifstream& in)
	{
		unsigned frame_size = 0;
		in.read((char*)&frame_size, sizeof(unsigned));
		if (frame_size <= 0)
		{
			in.close();
			return 0;
		}

		for (unsigned i = 0; i < frame_size; ++i)
		{
			float v[3];
			float m[9];
			unsigned id;
			float d;
			in.read((char*)&v[0], 3 * sizeof(float));
			in.read((char*)&id, sizeof(unsigned));
			in.read((char*)&v[0], 3 * sizeof(float));
			in.read((char*)&d, sizeof(float));
			in.read((char*)&m[0], 9 * sizeof(float));
		}

		unsigned point_size = 0;
		in.read((char*)&point_size, sizeof(unsigned));

		PointCloud* cloud = 0;
		if (point_size > 0)
		{
			cloud = new PointCloud();
			cloud->AllocateVertex(point_size);

			for (unsigned i = 0; i < point_size; ++i)
			{//frame_indice
				short index = 0;
				in.read((char*)&index, sizeof(short));
			}
			for (unsigned i = 0; i < point_size; ++i)
			{//point
				in.read((char*)(cloud->vertex_position + 3 * i), 3 * sizeof(float));
			}
			for (unsigned i = 0; i < point_size; ++i)
			{//point weight
				short sweight = 0;
				in.read((char*)&sweight, sizeof(short));
			}
			for (unsigned i = 0; i < point_size; ++i)
			{//normal
				in.read((char*)(cloud->vertex_normal + 3 * i), 3 * sizeof(float));
			}
			for (unsigned i = 0; i < point_size; ++i)
			{//point weight
				short sweight = 0;
				in.read((char*)&sweight, sizeof(short));
			}
			for (unsigned i = 0; i < point_size; ++i)
			{
				in.read((char*)(cloud->vertex_color + 3 * i), 3 * sizeof(unsigned char));
			}
		}
		in.close();

		return cloud;
	}

	PointCloud* PointCloudLoader::LoadFromNPTS(const std::wstring& mesh_file)
	{
		std::ifstream in;
		in.open(mesh_file, std::ios::in);
		if (!in.is_open())
			return 0;

		return LoadFromNPTS(in);
	}

	PointCloud* PointCloudLoader::LoadFromNPTS(const std::string& mesh_file)
	{
		std::ifstream in;
		in.open(mesh_file, std::ios::in);
		if (!in.is_open())
			return 0;

		return LoadFromNPTS(in);
	}

	PointCloud* PointCloudLoader::LoadFromNPTS(std::ifstream& in)
	{
		unsigned count = 0;

		std::vector<float> pos;
		std::vector<float> norn;
		while (!in.eof())
		{
			float p[3];
			float n[3];
			in >> (p[0]);
			in >> (p[1]);
			in >> (p[2]);
			in >> (n[0]);
			in >> (n[1]);
			in >> (n[2]);
			++count;

			pos.push_back(p[0]); pos.push_back(p[1]); pos.push_back(p[2]);
			norn.push_back(n[0]); norn.push_back(n[1]); norn.push_back(n[2]);
		}

		PointCloud* cloud = 0;
		
		count--;
		if (count > 0)
		{
			cloud  = new PointCloud();
			cloud->AllocateVertex(count);
			memcpy(cloud->vertex_position, &pos[0], 3 * count * sizeof(float));
			memcpy(cloud->vertex_normal, &norn[0], 3 * count * sizeof(float));
		}

		in.close();

		return cloud;
	}
}

