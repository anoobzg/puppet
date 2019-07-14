#pragma once
#include "EsslamBaseExport.h"
#include <vector>
#include  "../interface/slam_data.h"
#include "Box.h"

namespace esslam
{
	class ESSLAM_API DFrame
	{
	public:
		DFrame(int width, int height);
		~DFrame();

		BuildModelData data;

		trimesh::box3 box;
	};
}