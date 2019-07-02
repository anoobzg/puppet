#pragma once
#include "interface.h"
#include "csvwriter.h"

class LocateTracerImpl : public LocateTracer
{
public:
	LocateTracerImpl();
	virtual ~LocateTracerImpl();

	void OnFF();
	void OnBeforeF2F();
	void OnAfterF2F();
	void OnFM();
	void OnBeforeF2M();
	void OnAfterF2M();
	void OnRelocate();
	void OnBeforeRelocate();
	void OnAfterRelocate();

	void SaveFF(const std::string& file);
	void SaveFM(const std::string& file);
	void SaveRelocate(const std::string& file);

private:
	CSVWriter m_ff_writer;
	CSVWriter m_fm_writer;
	CSVWriter m_re_writer;
};