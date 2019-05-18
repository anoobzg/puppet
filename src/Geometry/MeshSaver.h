#ifndef GEOMETRY_MESH_SAVER
#define GEOMETRY_MESH_SAVER
#include "LauncaGeometryExport.h"
#include <string>

namespace LauncaGeometry
{
class Mesh;
class LAUNCA_GEOMETRY_API MeshSaver
{
public:
	static bool Save(const wchar_t* mesh_file, Mesh& mesh);
	static bool Save(const char* mesh_file, Mesh& mesh);
private:
	static bool SaveToBin(const std::wstring& mesh_file, Mesh& mesh);
	static bool SaveToSTL(const std::wstring& mesh_file, Mesh& mesh);
	static bool SaveToBin(const std::string& mesh_file, Mesh& mesh);
	static bool SaveToSTL(const std::string& mesh_file, Mesh& mesh);
	static bool SaveToSTL(std::fstream& stream, Mesh& mesh);
	static bool SaveToPLY(const std::string& mesh_file, Mesh& mesh);
	static bool SaveToPLY(const std::wstring& mesh_file, Mesh& mesh);
	static bool SaveToPLY(std::fstream& stream, Mesh& mesh);
};

}
#endif // GEOMETRY_MESH_SAVER