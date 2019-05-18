#ifndef SOUP_MESH_CREATOR
#define SOUP_MESH_CREATOR
#include "LauncaGeometryExport.h"
#include <string>

namespace LauncaGeometry
{
class Mesh;
class LAUNCA_GEOMETRY_API SoupMeshCreator
{
public:
	static Mesh* Create(Mesh& mesh);
};

}
#endif // SOUP_MESH_CREATOR