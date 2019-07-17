#include "TLocator.h"
#include "TData.h"
#include "DFrame.h"
#include <base\bind.h>

namespace esslam
{
	TLocator::TLocator()
		:base::Thread("TLocator"), m_visual(NULL)
	{
	}

	TLocator::~TLocator()
	{
	}

	void TLocator::StartLocate(const SlamParameters& parameters)
	{
		Start();
	}

	void TLocator::StopLocate()
	{
		Stop();
	}

	void TLocator::Locate(DFrame* frame)
	{
		task_runner()->PostTask(FROM_HERE, base::Bind(&TLocator::DoLocate, base::Unretained(this), frame));
	}

	void TLocator::DoLocate(DFrame* frame)
	{
		LocateData* locate_data = new LocateData();
		locate_data->lost = false;

#ifdef TRACE_SLAM
		frame->end_process = trimesh::now();
#endif
		m_visual->FrameLocated(frame, locate_data);
	}

	void TLocator::SetVisualTracer(IVisual* visual)
	{
		m_visual = visual;
	}
}