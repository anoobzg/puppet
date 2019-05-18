#ifndef SCAN_MESH_FILE
#define SCAN_MESH_FILE
#include "LauncaGeometryExport.h"
#include <fstream>

namespace LauncaGeometry
{
	class Mesh;
	class LAUNCA_GEOMETRY_API ScanFile2Mesh
	{
	public:
		static Mesh* Load(const wchar_t* file);
	
	private:
		static bool Count(const wchar_t* file, unsigned& count);
		static Mesh* _Load(const wchar_t* file, unsigned count);
	};
}
#endif // SCAN_MESH_FILE