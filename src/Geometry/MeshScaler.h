#ifndef GEOMETRY_MESH_SCALER
#define GEOMETRY_MESH_SCALER
#include "LauncaGeometryExport.h"
#include <string>

namespace LauncaGeometry
{
class Mesh;
class LAUNCA_GEOMETRY_API MeshScaler
{
public:
	static void Scale(float x, float y, float z, Mesh& mesh);
};

}
#endif // GEOMETRY_MESH_SCALER