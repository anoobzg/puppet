#include "TFusioner.h"

namespace esslam
{

	TFusioner::TFusioner()
		:base::Thread("TFusioner"), m_visual(NULL)
	{

	}

	TFusioner::~TFusioner()
	{

	}

	void TFusioner::StartFusion(const SlamParameters& parameters)
	{
		Start();
	}

	void TFusioner::StopFusion()
	{
		Stop();
	}

	void TFusioner::SetVisualTracer(IVisual* visual)
	{
		m_visual = visual;
	}
}