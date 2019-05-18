#ifndef TRANSFORM_LOADER
#define TRANSFORM_LOADER
#include "LauncaGeometryExport.h"
#include "GemTransform.h"
#include <string>

namespace LauncaGeometry
{
	class LAUNCA_GEOMETRY_API TransformLoader
	{
	public:
		static bool LoadFromFileName(GemTransform& transform, const wchar_t* file_name);
	};
}
#endif // TRANSFORM_LOADER