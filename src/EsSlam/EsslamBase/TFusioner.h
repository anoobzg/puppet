#pragma once
#include <base\threading\thread.h>
#include "DefaultModule.h"

namespace esslam
{
	class ESSLAM_API TFusioner : public base::Thread
	{
	public:
		TFusioner();
		virtual ~TFusioner();

		void StartFusion(const SlamParameters& parameters);
		void StopFusion();

		void SetVisualTracer(IVisual* visual);
	protected:
		IVisual* m_visual;
	};
}