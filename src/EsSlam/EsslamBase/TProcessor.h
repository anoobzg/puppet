#pragma once
#include <base\threading\thread.h>
#include "DefaultModule.h"

namespace esslam
{
	class ESSLAM_API TProcessor : public base::Thread, public DefaultProcessor
	{
	public:
		TProcessor();
		virtual ~TProcessor();

		void StartProcessor(const SlamParameters& parameters);
		void StopProcessor();

		void Build(IBuildTracer* tracer);
		void Clear();

		void ProcessFrame(DFrame* frame);
	protected:
		void InnerProcessFrame(DFrame* frame);
	};
}