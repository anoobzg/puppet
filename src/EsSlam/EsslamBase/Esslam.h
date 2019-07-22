#pragma once
#include "../interface/slam_interface.h"
#include "SlamParameters.h"
#include <base\synchronization\lock.h>

namespace esslam
{
	class Reader;
	class SlamVO;
	class RenderProxy;
	class DebugCenter;
	class ESSLAM_API Esslam : public IESSlam
	{
	public:
		Esslam();
		virtual ~Esslam();

		void SetupParameters(const SetupParameter& parameter);

		void Start();
		void Stop();

		void SetImageData(HandleScanImageData* data);
		void SetModelData(BuildModelData* data);
		HHScanData* GetScanData();
		void SetScanData(HHScanData* data);

		void Build(IBuildTracer* tracer);
		void Clear();

		void SetVisualTracer(IVisualTracer* tracer);
		void SetOSGTracer(IOSGTracer* tracer);
		void SetReadTracer(IReadTracer* tracer);
	private:
		void StartInner();
	protected:
		SlamParameters m_parameters;

		std::unique_ptr<Reader> m_reader;
		std::unique_ptr<SlamVO> m_vo;
		std::unique_ptr<RenderProxy> m_render_proxy;
		std::unique_ptr<DebugCenter> m_debug_center;

		ScanType m_scan_type;
		bool m_running;

		base::Lock m_state_lock;
	};
}