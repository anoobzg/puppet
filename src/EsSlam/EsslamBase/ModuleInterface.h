#pragma once
#include <memory>
#include "SlamParameters.h"

namespace esslam
{
	class IReadTracer;
	class IProcessor;
	struct HandleScanImageData;
	struct BuildModelData;
	class DFrame;

	class IInput
	{
	public:
		virtual ~IInput() {}
		virtual void SetInputTracer(IReadTracer* tracer) = 0;
		virtual void SetProcessor(IProcessor* processor) = 0;

		virtual void StartInput(const SlamParameters& parameters) = 0;
		virtual void StopInput() = 0;

		virtual void SetImageData(HandleScanImageData* data) = 0;

		virtual void Release(DFrame* frame) = 0;
	};

	class IVisual;
	class IBuildTracer;
	class IProcessor
	{
	public:
		virtual ~IProcessor() {}
		virtual void SetVisual(IVisual* visual) = 0;

		virtual void StartProcessor(const SlamParameters& parameters) = 0;
		virtual void StopProcessor() = 0;

		virtual void Build(IBuildTracer* tracer) = 0;
		virtual void Clear() = 0;

		virtual void ProcessFrame(DFrame* frame) = 0;
	};

	class IOSGTracer;
	class LocateData;
	class IVisual
	{
	public:
		virtual ~IVisual() {}
		virtual void SetOSGTracer(IOSGTracer* tracer) = 0;
		virtual void SetInput(IInput* input) = 0;

		virtual void StartVisual(const SlamParameters& parameters) = 0;
		virtual void StopVisual() = 0;

		virtual void FrameLocated(DFrame* frame, LocateData* locate_data) = 0;
	};
}