#pragma once
#include <string>
#include <vector>

namespace esslam
{
	struct PatchRenderData
	{
		std::vector<int> indices;
		std::vector<float> points;
		std::vector<float> normals;
		float xf[16];
		bool lost;
		int step;
	};

	class IVisualTracer
	{
	public:
		virtual ~IVisualTracer() {}

		virtual void OnFrame(PatchRenderData* data) = 0;
	};

	class IESSlam
	{
	public:
		virtual ~IESSlam() {}

		virtual bool Initialize() = 0;
		virtual void SetVisualTracer(IVisualTracer* tracer) = 0;

		virtual void StartSelfConsistent(const std::string& config_file) = 0;
		virtual void StartHandheld() = 0;
		virtual void Stop() = 0;
	};
}

typedef esslam::IESSlam* (*CreateSlamFunc)();
typedef void (*DestroySlamFunc)(esslam::IESSlam* slam);