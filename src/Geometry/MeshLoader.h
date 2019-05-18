#ifndef MESH_LOADER
#define MESH_LOADER
#include "LauncaGeometryExport.h"
#include <string>

namespace LauncaGeometry
{
	class Mesh;
	class LAUNCA_GEOMETRY_API MeshLoader
	{
	public:
		MeshLoader();
		~MeshLoader();

		static Mesh* LoadFromFileName(const wchar_t* file_name);
		static Mesh* LoadFromFileName(const char* file_name);
	private:
		static Mesh* LoadFromBin(const std::wstring& mesh_file);
		static Mesh* LoadFromSTL(const std::wstring& mesh_file);
		static Mesh* LoadFromBin(const std::string& mesh_file);
		static Mesh* LoadFromSTL(const std::string& mesh_file);
		static Mesh* LoadFromPLY(const std::wstring& mesh_file);
		static Mesh* LoadFromPLY(const std::string& mesh_file);
		static Mesh* LoadFromMBin(std::ifstream& in);
		static Mesh* LoadFromSTL(std::ifstream& in);
		static Mesh* LoadFromBin(std::ifstream& in);
	};
}
#endif // MESH_LOADER