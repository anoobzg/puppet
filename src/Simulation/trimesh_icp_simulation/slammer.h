#pragma once
#include "reader.h"
#include "slamvo.h"

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
	SlamParameters m_parameters;
};