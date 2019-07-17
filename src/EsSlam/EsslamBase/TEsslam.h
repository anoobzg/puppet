#pragma once
#include "BaseEsslam.h"
#include "TReader.h"
#include "TProcessor.h"
#include "TVisual.h"
#include "DFramePool.h"

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
		std::unique_ptr<TReader> m_t_reader;
		std::unique_ptr<TProcessor> m_t_processor;
		std::unique_ptr<TVisual> m_t_visual;

		DFramePool m_dframe_pool;
	};
}