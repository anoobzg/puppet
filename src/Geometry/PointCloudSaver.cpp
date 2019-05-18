#include "PointCloudSaver.h"
#include <fstream>
#include "PointCloud.h"

namespace LauncaGeometry
{

	bool PointCloudSaver::Save(const wchar_t* mesh_file, PointCloud& mesh)
	{
		std::wstring file = mesh_file;
		size_t pos = std::string::npos;
		if ((pos = file.rfind('.')) == std::string::npos)
		{
			return false;
		}

		std::string extension = std::string(file.begin() + pos, file.end());
		if (!strcmp(extension.c_str(), ".bin"))
			return SaveToBin(file, mesh);
		return false;
	}

	bool PointCloudSaver::Save(const char* mesh_file, PointCloud& mesh)
	{
		std::string file = mesh_file;
		size_t pos = std::string::npos;
		if ((pos = file.rfind('.')) == std::string::npos)
		{
			return false;
		}

		std::string extension = std::string(file.begin() + pos, file.end());
		if (!strcmp(extension.c_str(), ".bin"))
			return SaveToBin(file, mesh);
		return false;
	}

	bool PointCloudSaver::SaveToBin(const std::string& mesh_file, PointCloud& mesh)
	{
		std::fstream out;

		out.open(mesh_file.c_str(), std::ios::out | std::ios::binary);
		if (out.good())
		{
			return SaveToBin(out, mesh);
		}

		return false;
	}

	bool PointCloudSaver::SaveToBin(const std::wstring& mesh_file, PointCloud& mesh)
	{
		std::fstream out;

		out.open(mesh_file.c_str(), std::ios::out | std::ios::binary);
		if (out.good())
		{
			return SaveToBin(out, mesh);
		}

		return false;
	}

	bool PointCloudSaver::SaveToBin(std::fstream& file, PointCloud& point_cloud)
	{
		unsigned frame_size = 10;
		file.write((const char*)&frame_size, sizeof(unsigned));

		for (unsigned i = 0; i < frame_size; ++i)
		{
			float v[3] = { 0.0f };
			float m[9] = { 0.0f };
			unsigned id = 0;
			float d = 0.0f;
			file.write((const char*)&v[0], 3 * sizeof(float));
			file.write((const char*)&id, sizeof(unsigned));
			file.write((const char*)&v[0], 3 * sizeof(float));
			file.write((const char*)&d, sizeof(float));
			file.write((const char*)&m[0], 9 * sizeof(float));
		}

		unsigned point_size = point_cloud.vertex_number;
		file.write((const char*)&point_size, sizeof(unsigned));

		if (point_size > 0)
		{
			for (unsigned i = 0; i < point_size; ++i)
			{//frame_indice
				short index = 0;
				file.write((const char*)&index, sizeof(short));
			}
			
			file.write((const char*)point_cloud.vertex_position, 3 * point_size * sizeof(float));

			for (unsigned i = 0; i < point_size; ++i)
			{//point weight
				short sweight = 0;
				file.write((const char*)&sweight, sizeof(short));
			}

			file.write((const char*)point_cloud.vertex_normal, 3 * point_size * sizeof(float));

			for (unsigned i = 0; i < point_size; ++i)
			{//point weight
				short sweight = 0;
				file.write((const char*)&sweight, sizeof(short));
			}
			
			file.write((const char*)point_cloud.vertex_color, 3 * point_size * sizeof(unsigned char));
		}

		file.close();
		return true;
	}
}