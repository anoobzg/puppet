#include "Profiler.h"

namespace esslam
{

	Profiler Profiler::m_profiler = Profiler();
	Profiler::Profiler()
	{
		m_locate_writer.PushHead("index");
		m_locate_writer.PushHead("locate_s");
		m_locate_writer.PushHead("locate_e");

		m_fusion_writer.PushHead("index");
		m_fusion_writer.PushHead("fusion_s");
		m_fusion_writer.PushHead("fusion_e");
	}

	Profiler::~Profiler()
	{

	}

	Profiler& Profiler::Instance()
	{
		return m_profiler;
	}

	void Profiler::Start()
	{
		m_locate_writer.Start();
		m_fusion_writer.Start();
	}

	void Profiler::Output()
	{
		m_locate_writer.Output("locate.csv");
		m_fusion_writer.Output("fusion.csv");
	}
}