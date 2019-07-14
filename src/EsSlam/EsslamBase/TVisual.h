#pragma once
#include <base\threading\thread.h>
#include "DefaultModule.h"

namespace esslam
{
	class ESSLAM_API TVisual : public base::Thread, public DefaultVisual
	{
	public:
		TVisual();
		virtual ~TVisual();

		void StartVisual(const SlamParameters& parameters);
		void StopVisual();

		void FrameLocated(DFrame* frame);
	};
}