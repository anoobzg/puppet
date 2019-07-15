#include "TVisual.h"
#include "TData.h"
#include "DFrame.h"
#include "../interface/slam_osg.h"

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
		const BuildModelData& data = frame->data;
		if (m_tracer) m_tracer->OnFrameLocated(data.num_effective, data.points, data.normals, data.colors,
			locate_data->xf, locate_data->lost);

		m_input->Release(frame);
		delete locate_data;
	}
}