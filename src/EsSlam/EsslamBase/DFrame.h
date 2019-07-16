#pragma once
#include "EsslamBaseExport.h"
#include <vector>
#include  "../interface/slam_data.h"
#include "Box.h"
#include "timestamp.h"

namespace esslam
{
	class ESSLAM_API DFrame
	{
	public:
		DFrame(int width, int height);
		~DFrame();

		BuildModelData data;

		trimesh::box3 box;
		int index;  //index all
		int iindex; //index except invalid

#ifdef TRACE_SLAM
		trimesh::timestamp begin_read;
		trimesh::timestamp end_read;
		trimesh::timestamp begin_process;
		trimesh::timestamp end_process;
		trimesh::timestamp begin_visual;
		trimesh::timestamp end_visual;
#endif
	};
}