#include "slammer.h"

Slammer::Slammer()
{

}

Slammer::~Slammer()
{

}

void Slammer::Start(const std::string& config_file, VOTracer* tracer)
{
	m_parameters.LoadFromFile(config_file);

	const DebugParameters& debug_param = m_parameters.debug_param;
	if (debug_param.debug)
	{
		m_debug_center.reset(new DebugCenter());
		m_debug_center->SetParameters(debug_param);
		m_reader.SetReadTracer(m_debug_center->GetReadTracer());
		m_vo.SetVOProfiler(m_debug_center->GetVOProfiler());
		m_vo.SetLocateTracer(m_debug_center->GetLocateTracer());
	}

	m_reader.SetVO(&m_vo);
	m_vo.SetVOTracer(tracer);
	m_vo.StartVO(m_parameters);
	m_reader.StartRead(m_parameters.reader_param);
}

void Slammer::Stop()
{
	m_reader.StopRead();
	m_vo.StopVO();

	if (m_debug_center)
	{
		m_debug_center->Save();
		m_debug_center.reset();
	}
}