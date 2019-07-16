#pragma once
#include "../interface/slam_interface.h"
#include "ModuleInterface.h"
#include <base\synchronization\lock.h>
#include "DefaultModule.h"

namespace esslam
{
	class ESSLAM_API BaseEsslam : public IESSlam
	{
	public:
		BaseEsslam();
		virtual ~BaseEsslam();

		void SetupParameters(const SetupParameter& parameter);

		void Start();
		void Stop();

		void SetImageData(HandleScanImageData* data);
		void SetModelData(BuildModelData* data);

		void Build(IBuildTracer* tracer);
		void Clear();

		void SetVisualTracer(IVisualTracer* tracer);
		void SetOSGTracer(IOSGTracer* tracer);
		void SetReadTracer(IReadTracer* tracer);
	protected:
		virtual void OnStart();
		virtual void OnStop();
	private:
		void StartInner();
	protected:
		SlamParameters m_parameters;

		bool m_consistent_mode;
		bool m_running;

		base::Lock m_state_lock;

		IInput* m_input;
		IProcessor* m_processor;
		IVisual* m_visual;

		DefaultInput m_default_input;
		DefaultProcessor m_default_processor;
		DefaultVisual m_default_visual;
	};
}