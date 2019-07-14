#include "TProcessor.h"

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
		m_visual->FrameLocated(frame);
	}
}