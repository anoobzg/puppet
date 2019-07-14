#include "TVisual.h"

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

	void TVisual::FrameLocated(DFrame* frame)
	{
		m_input->Release(frame);
	}
}