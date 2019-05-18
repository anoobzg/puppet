#pragma once
#include "scene_logic_interface.h"
#include <map>

class PointGroup
{
public:
	PointGroup();
	~PointGroup();

	ControlPoint* Create(unsigned handle);
	ControlPoint* Get(unsigned handle);
	void Delete(unsigned handle);
	bool TestOverlap(unsigned vertex_handle, XYZ position);

	const std::map<unsigned, ControlPoint*>& Get() const;
private:
	std::map<unsigned, ControlPoint*> m_points;
};