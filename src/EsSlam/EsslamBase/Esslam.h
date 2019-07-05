#pragma once
#include "../interface/slam_interface.h"
#include "SlamParameters.h"

namespace esslam
{
	class Reader;
	class SlamVO;
	class DebugCenter;
	class ESSLAM_API Esslam : public IESSlam
	{
	public:
		Esslam();
		virtual ~Esslam();

		bool Initialize();

		void SetVisualTracer(IVisualTracer* tracer);

		void StartSelfConsistent(const std::string& config_file);
		void StartHandheld();
		void Stop();
	private:
		void StartInner(const std::string& file);
	protected:
		SlamParameters m_parameters;

		std::unique_ptr<Reader> m_reader;
		std::unique_ptr<SlamVO> m_vo;
		std::unique_ptr<DebugCenter> m_debug_center;

		bool m_consistent_mode;
		bool m_running;
	};
}