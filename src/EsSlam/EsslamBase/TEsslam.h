#pragma once
#include "BaseEsslam.h"
#include "TReader.h"
#include "TProcessor.h"
#include "TVisual.h"

namespace esslam
{
	class ESSLAM_API TEsslam : public BaseEsslam
	{
	public:
		TEsslam();
		virtual ~TEsslam();

	protected:
		void OnStart();
		void OnStop();
	private:
		TReader m_t_reader;
		TProcessor m_t_processor;
		TVisual m_t_visual;
	};
}