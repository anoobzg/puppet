#pragma once
#include "readtracerimpl.h"
#include "voprofilerimpl.h"
#include "locatetracerimpl.h"

class DebugCenter
{
public:
	DebugCenter();
	~DebugCenter();

	void SetParameters(const DebugParameters& parameters);
	void Save();
	ReadTracer* GetReadTracer();
	VOProfiler* GetVOProfiler();
	LocateTracer* GetLocateTracer();
protected:
	std::string TimeLine();
private:
	ReadTracerImpl m_read_tracer;
	VOProfilerImpl m_vo_profiler;
	LocateTracerImpl m_locate_tracer;
	std::string m_debug_directory;
};