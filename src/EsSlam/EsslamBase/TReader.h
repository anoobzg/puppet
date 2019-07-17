#pragma once
#include <base\threading\thread.h>
#include "DefaultModule.h"

#include "DirectoryReader.h"
#include <base/synchronization/lock.h>
#include "timestamp.h"
#include "DFramePool.h"

namespace esslam
{
	class IReadTracer;
	class ESSLAM_API TReader : public base::Thread, public DefaultInput
	{
	public:
		TReader(DFramePool& pool);
		virtual ~TReader();

		void StartInput(const SlamParameters& parameters);
		void StopInput();

		void SetImageData(HandleScanImageData* data);
		void Release(DFrame* frame);
	private:
		void Read();
	protected:
		DirectoryReader m_reader;
		trimesh::timestamp m_last_time;
		float m_delta_time;

		DFramePool& m_dframe_pool;
	};
}