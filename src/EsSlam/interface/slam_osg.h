#pragma once
#include "Vec.h"
#include <vector>
#include "Xform.h"

namespace esslam
{
	class IOSGTracer
	{
	public:
		virtual ~IOSGTracer() {}

		virtual void OnFrameLocated(int effect_num, const std::vector<trimesh::point3>& vertexes, 
			const std::vector<trimesh::point3>& normals, const std::vector<trimesh::point3>& colors,
			const trimesh::xform& xf, bool lost) = 0;
	};
} 