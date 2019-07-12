#pragma once
#include <base\threading\thread.h>
#include "SlamParameters.h"
#include "InnerInterface.h"

namespace esslam
{
	class IReadTracer;
	class ESSLAM_API Reader : public base::Thread
	{
	public:
		Reader();
		virtual ~Reader();

		void StartRead(const ReaderParameters& parameters);
		void StopRead();
		void SetVO(VO* vo);
		void SetReadTracer(IReadTracer* tracer);
	private:
		void Read();
		trimesh::TriMesh* LoadOneFrame();
	protected:
		ReaderParameters m_parameters;
		int m_current_index;
		bool m_stop;

		VO* m_vo;
		IReadTracer* m_tracer;
	};
}