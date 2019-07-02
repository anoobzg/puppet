#include "debugcenter.h"
#include <boost\filesystem.hpp>

DebugCenter::DebugCenter()
{

}

DebugCenter::~DebugCenter()
{

}

void DebugCenter::SetParameters(const DebugParameters& parameters)
{
	std::string prefix = TimeLine();
	m_debug_directory = parameters.directory + "\\" + prefix + "\\";
	if (!boost::filesystem::exists(m_debug_directory))
		boost::filesystem::create_directories(m_debug_directory);
}

ReadTracer* DebugCenter::GetReadTracer()
{
	return &m_read_tracer;
}

VOProfiler* DebugCenter::GetVOProfiler()
{
	return &m_vo_profiler;
}

LocateTracer* DebugCenter::GetLocateTracer()
{
	return &m_locate_tracer;
}

void DebugCenter::Save()
{
	std::string read_csv_file = m_debug_directory + "read.csv";
	std::string icp_csv_file = m_debug_directory + "icp.csv";
	std::string ff_csv_file = m_debug_directory + "ff.csv";
	std::string fm_csv_file = m_debug_directory + "fm.csv";
	std::string relocate_csv_file = m_debug_directory + "relocate.csv";

	m_read_tracer.Write(read_csv_file);
	m_vo_profiler.Write(icp_csv_file);
	m_locate_tracer.SaveFF(ff_csv_file);
	m_locate_tracer.SaveFM(fm_csv_file);
	m_locate_tracer.SaveRelocate(relocate_csv_file);
}

std::string DebugCenter::TimeLine()
{
	char buf[128] = { 0 };
	time_t t = time(NULL);
	tm* local = localtime(&t);
	strftime(buf, 64, "%Y-%m-%d-%H-%M-%S", local);
	return std::string(buf);
}