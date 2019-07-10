#pragma once
#include "EsslamBaseExport.h"
#include <base\threading\thread.h>
#include "..\interface\slam_tracer.h"
#include "InnerInterface.h"

namespace esslam
{
	class ESSLAM_API RenderProxy : public base::Thread, public VisualProcessor
	{
	public:
		RenderProxy();
		virtual ~RenderProxy();

		void SetVisualTracer(IVisualTracer* tracer);

		void StartProcess();
		void StopProcess();

		void OnCurrentFrame(CurrentFrameData* data);
		void OnAppendNewPoints(NewAppendData* data);
	private:
		void ProcessCurrentFrame(CurrentFrameData* data);
		void ProcessNewData(NewAppendData* data);
	protected:
		IVisualTracer* m_visual_tracer;
	};
}