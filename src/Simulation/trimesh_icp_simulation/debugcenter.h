#pragma once
#include "readtracerimpl.h"
#include "voprofilerimpl.h"

class DebugCenter
{
public:
	DebugCenter();
	~DebugCenter();

	void SetParameters(const DebugParameters& parameters);
	void Save();
	ReadTracer* GetReadTracer();
	VOProfiler* GetVOProfiler();
protected:
	std::string TimeLine();
private:
	ReadTracerImpl m_read_tracer;
	VOProfilerImpl m_vo_profiler;
	std::string m_debug_directory;
};