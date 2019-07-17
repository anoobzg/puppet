#include "TVisual.h"
#include "TData.h"
#include "DFrame.h"
#include "../interface/slam_osg.h"
#include <base\bind.h>

namespace esslam
{
	TVisual::TVisual()
		:base::Thread("TVisual")
	{

	}

	TVisual::~TVisual()
	{

	}

	void TVisual::StartVisual(const SlamParameters& parameters)
	{
		bool start = Start();
	}

	void TVisual::StopVisual()
	{
		Stop();
	}

	void TVisual::FrameLocated(DFrame* frame, LocateData* locate_data)
	{
		task_runner()->PostTask(FROM_HERE, base::Bind(&TVisual::ProcessFrameLocated, base::Unretained(this), frame, locate_data));
	}

	void TVisual::ProcessFrameLocated(DFrame* frame, LocateData* locate_data)
	{
		const BuildModelData& data = frame->data;

#ifdef TRACE_SLAM
		frame->begin_visual = trimesh::now();
#endif
		if (m_tracer) m_tracer->OnFrameLocated(data.num_effective, data.points, data.normals, data.colors,
			locate_data->xf, locate_data->lost);

#ifdef TRACE_SLAM
		frame->end_visual = trimesh::now();
#endif

		m_input->Release(frame);
		delete locate_data;

#ifdef TRACE_SLAM
		std::cout << frame->data.num_effective<<" Read " << (frame->end_read - frame->begin_read) << " Process " << (frame->end_process - frame->begin_process)
			<< " Visual " << (frame->end_visual - frame->begin_visual) << std::endl;
#endif
	}
}