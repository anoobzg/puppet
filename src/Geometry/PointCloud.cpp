#include "PointCloud.h"

namespace LauncaGeometry
{
	PointCloud::PointCloud()
		:vertex_number(0), vertex_position(0), vertex_normal(0), vertex_color(0)
	{
	}

	PointCloud::~PointCloud()
	{
		ReleaseVertex();
	}

	void PointCloud::AllocateVertex(unsigned number)
	{
		ReleaseVertex();

		vertex_number = number;
		vertex_position = new float[3 * vertex_number];
		vertex_normal = new float[3 * vertex_number];
		vertex_color = new unsigned char[3 * vertex_number];
	}

	void PointCloud::ReleaseVertex()
	{
		vertex_number = 0;
		if (vertex_position) { delete[] vertex_position; vertex_position = 0; }
		if (vertex_normal) { delete[] vertex_normal; vertex_normal = 0; }
		if (vertex_color) { delete[] vertex_color; vertex_color = 0; }
	}

}