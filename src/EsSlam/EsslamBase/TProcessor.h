#pragma once
#include <base\threading\thread.h>
#include "DefaultModule.h"
#include "TLocator.h"
#include "TFusioner.h"

namespace esslam
{
	class ESSLAM_API TProcessor : public DefaultProcessor
	{
	public:
		TProcessor();
		virtual ~TProcessor();

		void StartProcessor(const SlamParameters& parameters);
		void StopProcessor();

		void Build(IBuildTracer* tracer);
		void Clear();

		void ProcessFrame(DFrame* frame);
	private:
		TLocator m_locator;
		TFusioner m_fusioner;
	};
}