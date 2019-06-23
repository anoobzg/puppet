#include "vector_util.h"

namespace trimesh
{

	float vector_mean(const std::vector<float>& values)
	{
		size_t n = values.size();
		if (!n)
			return 0.0f;

		float sum = 0.0f;
		for (size_t i = 0; i < n; i++)
			sum += values[i];

		return sum / n;
	}

}