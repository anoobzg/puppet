#include "TProcessor.h"
#include <base\bind.h>

namespace esslam
{
	TProcessor::TProcessor()
	{

	}

	TProcessor::~TProcessor()
	{

	}

	void TProcessor::StartProcessor(const SlamParameters& parameters)
	{
		m_locator.SetVisualTracer(m_visual);
		m_fusioner.SetVisualTracer(m_visual);
		m_locator.StartLocate(parameters);
		m_fusioner.StartFusion(parameters);
	}

	void TProcessor::StopProcessor()
	{
		m_locator.StopLocate();
		m_fusioner.StopFusion();
	}

	void TProcessor::Build(IBuildTracer* tracer)
	{

	}

	void TProcessor::Clear()
	{

	}

	void TProcessor::ProcessFrame(DFrame* frame)
	{
		m_locator.Locate(frame);
	}
}