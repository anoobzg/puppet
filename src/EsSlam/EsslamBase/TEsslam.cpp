#include "TEsslam.h"

namespace esslam
{
	TEsslam::TEsslam()
	{
		m_t_reader.reset(new TReader(m_dframe_pool));
		m_t_processor.reset(new TProcessor());
		m_t_visual.reset(new TVisual());
	}

	TEsslam::~TEsslam()
	{

	}

	void TEsslam::OnStart()
	{
		const ImageParameters& image_param = m_parameters.image_param;
		m_dframe_pool.Melloc(3, image_param.width, image_param.height);

		if(m_parameters.reader_param.load_from_file)
			m_input = m_t_reader.get();
		m_processor = m_t_processor.get();
		m_visual = m_t_visual.get();
	}

	void TEsslam::OnStop()
	{
		m_dframe_pool.Free();
	}
}