#include "readtracerimpl.h"

ReadTracerImpl::ReadTracerImpl()
{
	m_writer.PushHead("time");
}

ReadTracerImpl::~ReadTracerImpl()
{

}

void ReadTracerImpl::OnBeforeRead()
{
	m_writer.TickStart();
}

void ReadTracerImpl::OnAfterRead()
{
	m_writer.TickEnd();
}

void ReadTracerImpl::Write(const std::string& file)
{
	m_writer.Output(file);
}