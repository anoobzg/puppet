#include "locatetracerimpl.h"

LocateTracerImpl::LocateTracerImpl()
{
	m_ff_writer.PushHead("time");
	m_fm_writer.PushHead("time");
	m_re_writer.PushHead("time");
}

LocateTracerImpl::~LocateTracerImpl()
{

}

void LocateTracerImpl::OnFF()
{

}

void LocateTracerImpl::OnBeforeF2F()
{
	m_ff_writer.TickStart();
}

void LocateTracerImpl::OnAfterF2F()
{
	m_ff_writer.TickEnd();
}

void LocateTracerImpl::OnFM()
{

}

void LocateTracerImpl::OnBeforeF2M()
{
	m_fm_writer.TickStart();
}

void LocateTracerImpl::OnAfterF2M()
{
	m_fm_writer.TickEnd();
}

void LocateTracerImpl::OnRelocate()
{

}

void LocateTracerImpl::OnBeforeRelocate()
{
	m_re_writer.TickStart();
}

void LocateTracerImpl::OnAfterRelocate()
{
	m_re_writer.TickEnd();
}

void LocateTracerImpl::SaveFF(const std::string& file)
{
	m_ff_writer.Output(file);
}

void LocateTracerImpl::SaveFM(const std::string& file)
{
	m_fm_writer.Output(file);
}

void LocateTracerImpl::SaveRelocate(const std::string& file)
{
	m_re_writer.Output(file);
}