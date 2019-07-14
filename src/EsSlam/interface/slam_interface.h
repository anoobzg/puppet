#pragma once
#include <string>

namespace esslam
{
	enum ScanType
	{
		e_load_from_file,
		e_handheld,
		e_fix
	};

	struct SetupParameter
	{
		ScanType type;
		std::string default_config;
		std::string calib_file;
	};

	struct BuildModelData;
	struct HandleScanImageData;
	class IBuildTracer;
	class IVisualTracer;
	class IOSGTracer;
	class IReadTracer;
	class IESSlam
	{
	public:
		virtual ~IESSlam() {}

		virtual void SetupParameters(const SetupParameter& parameter) = 0;
		virtual void Start() = 0;
		virtual void Stop() = 0;

		virtual void SetImageData(HandleScanImageData* data) = 0;
		virtual void SetModelData(BuildModelData* data) = 0;

		virtual void Build(IBuildTracer* tracer) = 0;
		virtual void Clear() = 0;

		virtual void SetVisualTracer(IVisualTracer* tracer) = 0; //old visual interface
		virtual void SetOSGTracer(IOSGTracer* tracer) = 0; //osg visual interface
		virtual void SetReadTracer(IReadTracer* tracer) = 0;
	};
}

typedef esslam::IESSlam* (*CreateSlamFunc)(const char* name);
typedef void (*DestroySlamFunc)(esslam::IESSlam* slam);