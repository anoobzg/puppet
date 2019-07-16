#include "TEsslam.h"

namespace esslam
{
	TEsslam::TEsslam()
	{
		m_input = &m_t_reader;
		m_processor = &m_t_processor;
		m_visual = &m_t_visual;
	}

	TEsslam::~TEsslam()
	{

	}

	void TEsslam::OnStart()
	{

	}

	void TEsslam::OnStop()
	{

	}
}