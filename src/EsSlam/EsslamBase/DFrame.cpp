#include "DFrame.h"

namespace esslam
{
	DFrame::DFrame(int width, int height)
	{
		data.width = width;
		data.height = height;
		int size = width * height;
		data.grid.resize(size, -1);
		data.normals.resize(size);
		data.points.resize(size);
		data.colors.resize(size);
		data.num_effective = 0;
		index = -1;
		iindex = - 1;
	}

	DFrame::~DFrame()
	{
		data.grid.swap(std::vector<int>());
		data.points.swap(std::vector<trimesh::point3>());
		data.normals.swap(std::vector<trimesh::point3>());
		data.colors.swap(std::vector<trimesh::point3>());
	}
}