#pragma once
#include "interface.h"
#include "csvwriter.h"

class ReadTracerImpl : public ReadTracer
{
public:
	ReadTracerImpl();
	virtual ~ReadTracerImpl();

	void OnBeforeRead();
	void OnAfterRead();

	void Write(const std::string& file);
protected:
	CSVWriter m_writer;
};