#pragma once
#include "LauncaGeometryExport.h"
#include <string>

namespace LauncaGeometry
{
	class PointCloud;
	class LAUNCA_GEOMETRY_API PointCloudSaver
	{
	public:
		static bool Save(const wchar_t* mesh_file, PointCloud& mesh);
		static bool Save(const char* mesh_file, PointCloud& mesh);
	private:
		static bool SaveToBin(const std::wstring& mesh_file, PointCloud& mesh);
		static bool SaveToBin(const std::string& mesh_file, PointCloud& mesh);
		static bool SaveToBin(std::fstream& file, PointCloud& mesh);
	};

}
