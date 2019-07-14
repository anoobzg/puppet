#pragma once
#include "ModuleInterface.h"

namespace esslam
{
	class DefaultInput : public IInput
	{
	public:
		DefaultInput();
		virtual ~DefaultInput();

		void SetInputTracer(IReadTracer* tracer);
		void SetProcessor(IProcessor* processor);

		void StartInput(const SlamParameters& parameters);
		void StopInput();

		void SetImageData(HandleScanImageData* data);

		void Release(DFrame* frame);
	protected:
		IReadTracer* m_read_tracer;
		IProcessor* m_processor;
	};

	class DefaultProcessor : public IProcessor
	{
	public:
		DefaultProcessor();
		virtual ~DefaultProcessor();
		void SetVisual(IVisual* visual);

		void StartProcessor(const SlamParameters& parameters);
		void StopProcessor();

		void Build(IBuildTracer* tracer);
		void Clear();

		void ProcessFrame(DFrame* frame);

	protected:
		IVisual* m_visual;
	};

	class DefaultVisual : public IVisual
	{
	public:
		DefaultVisual();
		virtual ~DefaultVisual();
		void SetOSGTracer(IOSGTracer* tracer);
		void SetInput(IInput* input);

		void StartVisual(const SlamParameters& parameters);
		void StopVisual();

		void FrameLocated(DFrame* frame);
	protected:
		IOSGTracer* m_tracer;
		IInput* m_input;
	};
}