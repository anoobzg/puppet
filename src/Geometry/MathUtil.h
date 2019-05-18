#pragma once
#ifndef MATH_UTIL
#define MATH_UTIL
#include "LauncaGeometryExport.h"

namespace LauncaGeometry
{
	class LAUNCA_GEOMETRY_API MathUtil
	{
	public:
		static void Cross(float* n1, float* n2, float* result);
		static void Normalize(float* n);
	};
}
#endif // MATH_UTIL