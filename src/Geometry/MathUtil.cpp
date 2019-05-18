#include "MathUtil.h"
#include <math.h>

namespace LauncaGeometry
{
	void MathUtil::Cross(float* n1, float* n2, float* result)
	{
		result[0] = n1[1] * n2[2] - n1[2] * n2[1];
		result[1] = n1[2] * n2[0] - n1[0] * n2[2];
		result[2] = n1[1] * n2[0] - n1[0] * n2[1];
 	}

	void MathUtil::Normalize(float* n)
	{
		float l = sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
		if (l > 0.0f)
		{
			n[0] /= l;
			n[1] /= l;
			n[2] /= l;
		}
	}
}