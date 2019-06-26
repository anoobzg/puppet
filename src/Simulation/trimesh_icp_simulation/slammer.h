#pragma once
#include "reader.h"
#include "slamvo.h"
#include "debugcenter.h"

class Slammer
{
public:
	Slammer();
	~Slammer();

	void Start(const std::string& config_file, VOTracer* tracer = NULL);
	void Stop();
private:
	Reader m_reader;
	SlamVO m_vo;
	std::unique_ptr<DebugCenter> m_debug_center;

	SlamParameters m_parameters;
};