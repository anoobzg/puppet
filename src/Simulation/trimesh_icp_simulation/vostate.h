#pragma once

class VOState
{
public:
	VOState();
	~VOState();

	inline bool FirstFrame() { return m_first_frame; }
	inline void SetFirstFrame(bool first) { m_first_frame = first; }
protected:
	bool m_first_frame;
};