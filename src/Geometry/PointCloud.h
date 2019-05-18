#pragma once
#include "LauncaGeometryExport.h"

namespace LauncaGeometry
{
	class LAUNCA_GEOMETRY_API PointCloud
	{
	public:
		PointCloud();
		~PointCloud();

		void AllocateVertex(unsigned vertex_number);

		unsigned vertex_number;
		float*   vertex_position;
		float*   vertex_normal;
		unsigned char* vertex_color;
	private:
		void ReleaseVertex();
	};
}