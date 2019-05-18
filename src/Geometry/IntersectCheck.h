#pragma once
#include "LauncaGeometryExport.h"
namespace LauncaGeometry 
{
	class LAUNCA_GEOMETRY_API IntersectChecker
	{
	public:
		bool Intersect(float* rayCenter, float* rayDirection, float* vertices);
	};
}