#pragma once
#include <base\threading\thread.h>
#include "DefaultModule.h"

namespace esslam
{
	class DFrame;
	class ESSLAM_API TLocator : public base::Thread
	{
	public:
		TLocator();
		virtual ~TLocator();

		void StartLocate(const SlamParameters& parameters);
		void StopLocate();

		void Locate(DFrame* frame);

		void SetVisualTracer(IVisual* visual);
	protected:
		void DoLocate(DFrame* frame);

	private:
		IVisual* m_visual;
	};
}