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

	m_reader.SetVO(&m_vo);
	m_vo.SetVOTracer(tracer);
	m_vo.StartVO(m_parameters.icp_param);
	m_reader.StartRead(m_parameters.reader_param);
}

void Slammer::Stop()
{
	m_reader.StopRead();
	m_vo.StopVO();
}