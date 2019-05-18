#pragma once
#include "LauncaGeometryExport.h"
#include <string>

namespace LauncaGeometry
{
	class PointCloud;
	class LAUNCA_GEOMETRY_API PointCloudLoader
	{
	public:
		PointCloudLoader();
		~PointCloudLoader();

		static PointCloud* LoadFromFileName(const wchar_t* file_name);
		static PointCloud* LoadFromFileName(const char* file_name);
	private:
		static PointCloud* LoadFromBin(const std::wstring& mesh_file);
		static PointCloud* LoadFromBin(const std::string& mesh_file);
		static PointCloud* LoadFromBin(std::ifstream& in);
		static PointCloud* LoadFromNPTS(const std::wstring& mesh_file);
		static PointCloud* LoadFromNPTS(const std::string& mesh_file);
		static PointCloud* LoadFromNPTS(std::ifstream& in);
	};
}