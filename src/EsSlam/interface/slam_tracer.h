#pragma once

namespace esslam
{
	struct PatchRenderData;
	struct FrameData;
	struct NewAppendData;
	class IVisualTracer
	{
	public:
		virtual ~IVisualTracer() {}

		virtual void OnFrame(PatchRenderData* data) = 0;
		virtual void OnFrameLocated(FrameData* data) = 0;
		virtual void OnNewPoints(NewAppendData* data) = 0;
	};

	class IBuildTracer
	{
	public:
		virtual ~IBuildTracer() {}
		virtual void OnPoints(int size, float* position, float* normal, unsigned char* color) = 0;
	};

	class IReadTracer
	{
	public:
		virtual ~IReadTracer() {}
		virtual void OnFinished() = 0;
	};
}