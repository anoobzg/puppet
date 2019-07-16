#include "TProcessor.h"
#include "TData.h"
#include <base\bind.h>
#include "DFrame.h"

namespace esslam
{
	TProcessor::TProcessor()
		:base::Thread("TProcessor")
	{

	}

	TProcessor::~TProcessor()
	{

	}

	void TProcessor::StartProcessor(const SlamParameters& parameters)
	{
		bool start = Start();
	}

	void TProcessor::StopProcessor()
	{
		Stop();
	}

	void TProcessor::Build(IBuildTracer* tracer)
	{

	}

	void TProcessor::Clear()
	{

	}

	void TProcessor::ProcessFrame(DFrame* frame)
	{
		task_runner()->PostTask(FROM_HERE, base::Bind(&TProcessor::InnerProcessFrame, base::Unretained(this), frame));
	}

	void TProcessor::InnerProcessFrame(DFrame* frame)
	{
		LocateData* locate_data = new LocateData();
		locate_data->lost = false;

#ifdef TRACE_SLAM
		frame->end_process = trimesh::now();
		frame->begin_visual = trimesh::now();
#endif
		m_visual->FrameLocated(frame, locate_data);
	}
}