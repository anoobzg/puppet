#include "BaseEsslam.h"
#include "../interface/slam_data.h"
namespace esslam
{
	BaseEsslam::BaseEsslam()
		:m_consistent_mode(true), m_running(false),m_input(NULL), m_processor(NULL), m_visual(NULL)
		, m_visual_tracer(NULL), m_osg_tracer(NULL), m_read_tracer(NULL)
	{
	}

	BaseEsslam::~BaseEsslam()
	{

	}

	void BaseEsslam::SetupParameters(const SetupParameter& parameter)
	{
		m_state_lock.Acquire();
		if (!m_running)
		{
			m_consistent_mode = parameter.type == e_load_from_file;
			const std::string& file = parameter.default_config;
			m_parameters.LoadFromFile(file);
			if (!m_consistent_mode)
			{
				m_parameters.icp_param.calib_file = parameter.calib_file;
			}
			else
			{
				m_parameters.reader_param.load_from_file = true;
			}
		}
		m_state_lock.Release();
	}

	void BaseEsslam::SetVisualTracer(IVisualTracer* tracer)
	{
		//empty
	}

	void BaseEsslam::SetOSGTracer(IOSGTracer* tracer)
	{
		m_state_lock.Acquire();
		if (!m_running) m_osg_tracer = tracer;
		m_state_lock.Release();
	}

	void BaseEsslam::SetReadTracer(IReadTracer* tracer)
	{
		m_state_lock.Acquire();
		if (!m_running) m_read_tracer = tracer;
		m_state_lock.Release();
	}

	void BaseEsslam::Start()
	{
		m_state_lock.Acquire();
		m_running = true;
		m_state_lock.Release();

		OnStart();

		//make sure m_input, m_processor, m_visual are not null
		if (!m_input) m_input = &m_default_input;
		if (!m_processor) m_processor = &m_default_processor;
		if (!m_visual) m_visual = &m_default_visual;

		StartInner();
	}

	void BaseEsslam::StartInner()
	{
		m_visual->SetOSGTracer(m_osg_tracer);
		m_input->SetInputTracer(m_read_tracer);
		m_visual->StartVisual(m_parameters);
		m_visual->SetInput(m_input);
		m_processor->SetVisual(m_visual);
		m_processor->StartProcessor(m_parameters);
		m_input->SetProcessor(m_processor);
		m_input->StartInput(m_parameters);
	}

	void BaseEsslam::Stop()
	{
		m_state_lock.Acquire();
		m_running = false;
		m_state_lock.Release();

		m_input->StopInput();
		m_processor->StopProcessor();
		m_visual->StopVisual();

		OnStop();
	}

	void BaseEsslam::Clear()
	{
		m_state_lock.Acquire();
		if (!m_running) m_processor->Clear();
		m_state_lock.Release();	
	}

	void BaseEsslam::SetImageData(HandleScanImageData* data)
	{
		m_state_lock.Acquire();
		if (m_running) m_input->SetImageData(data);
		else delete data;
		m_state_lock.Release();
	}

	void BaseEsslam::SetModelData(BuildModelData* data)
	{

	}

	HHScanData* BaseEsslam::GetScanData()
	{
		return NULL;
	}

	void BaseEsslam::SetScanData(HHScanData* data)
	{
		
	}

	void BaseEsslam::Build(IBuildTracer* tracer)
	{
		m_state_lock.Acquire();
		if (!m_running && tracer) m_processor->Build(tracer);
		m_state_lock.Release();
	}

	void BaseEsslam::OnStart()
	{

	}

	void BaseEsslam::OnStop()
	{

	}
}