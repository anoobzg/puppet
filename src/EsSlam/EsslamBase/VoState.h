#pragma once
#include "EsslamBaseExport.h"

namespace esslam
{

	class ESSLAM_API VOState
	{
	public:
		VOState();
		~VOState();

		inline void IncFrame() { ++m_frame; }
		inline int Frame() { return m_frame; }
		inline bool FirstFrame() { return m_first_frame; }
		inline void SetFirstFrame(bool first) { m_first_frame = first; }
		inline bool RelocationState() { return m_lost_times > 2; }
		inline void ResetRelocation() { m_lost_times = 0; }
		inline void IncreLostTimes() { ++m_lost_times; }
		inline void Reset() { m_first_frame = true; m_frame = -1; m_lost_times = 0; }
	protected:
		bool m_first_frame;
		int m_frame;
		int m_lost_times;
	};
}