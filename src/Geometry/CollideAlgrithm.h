#pragma once
#include "LauncaGeometryExport.h"

namespace LauncaGeometry
{
	class LAUNCA_GEOMETRY_API ColliderAlgrithm
	{
	public:
		// ray collide triangle
		// default uvw & intersectCoord = {0.0f, 0.0f, 0.0f}
		static bool RayCollideTriangle(float* ray_position, float* ray_direction, float* v0, float* v1, float* v2,
			/* out */float* uvw, /* out */float* intersectCoord);
	};
}