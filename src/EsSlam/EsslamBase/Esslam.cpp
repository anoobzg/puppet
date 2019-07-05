#include "Esslam.h"
#include "Reader.h"
#include "SlamVo.h"
#include "DebugCenter.h"

namespace esslam
{
	Esslam::Esslam()
		:m_consistent_mode(true), m_running(false)
	{
		m_reader.reset(new Reader());
		m_vo.reset(new SlamVO());
	}

	Esslam::~Esslam()
	{

	}

	bool Esslam::Initialize()
	{
		return true;
	}

	void Esslam::SetVisualTracer(IVisualTracer* tracer)
	{
		if (m_running) return;
		m_vo->SetVisualTracer(tracer);
	}

	void Esslam::StartSelfConsistent(const std::string& config_file)
	{
		m_consistent_mode = true;
		StartInner(config_file);
	}

	void Esslam::StartHandheld()
	{
		m_consistent_mode = false;
		std::string default_file;
		StartInner(default_file);
	}

	void Esslam::StartInner(const std::string& file)
	{
		m_running = true;

		m_parameters.LoadFromFile(file);
		const DebugParameters& debug_param = m_parameters.debug_param;
		if (debug_param.debug)
		{
			m_debug_center.reset(new DebugCenter());
			m_debug_center->SetParameters(debug_param);
			m_vo->SetVOProfiler(m_debug_center->GetVOProfiler());
			m_vo->SetLocateTracer(m_debug_center->GetLocateTracer());
			m_vo->SetICPTracer(m_debug_center->GetProjectionICPTracer());
		}

		m_vo->StartVO(m_parameters);
		if (m_consistent_mode)
		{
			m_reader->SetVO(m_vo.get());
			m_reader->StartRead(m_parameters.reader_param);
		}
	}

	void Esslam::Stop()
	{
		m_running = false;
		if (m_consistent_mode)
			m_reader->StopRead();

		m_vo->StopVO();

		if (m_debug_center)
		{
			m_debug_center->Save();
			m_debug_center.reset();
		}
	}
}