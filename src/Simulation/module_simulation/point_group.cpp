#include "point_group.h"

PointGroup::PointGroup()
{

}

PointGroup::~PointGroup()
{

}

ControlPoint* PointGroup::Create(unsigned handle)
{
	ControlPoint* point = new ControlPoint();
	point->m_vertex_index = handle;

	std::pair<std::map<unsigned, ControlPoint*>::iterator, bool> result = m_points.insert(std::pair<unsigned, ControlPoint*>(point->m_handle, point));
	if (result.second == false)
	{
		delete point;
		point = 0;
	}

	return point;
}

ControlPoint* PointGroup::Get(unsigned handle)
{
	std::map<unsigned, ControlPoint*>::iterator result = m_points.find(handle);
	if (result == m_points.end())
		return 0;

	ControlPoint* control_point = result->second;
	return control_point;
}

void PointGroup::Delete(unsigned handle)
{
	std::map<unsigned, ControlPoint*>::iterator result = m_points.find(handle);
	if (result == m_points.end())
		return;

	ControlPoint* control_point = result->second;
	m_points.erase(result);
	delete control_point;
}

const std::map<unsigned, ControlPoint*>& PointGroup::Get() const
{
	return m_points;
}

bool PointGroup::TestOverlap(unsigned vertex_handle, XYZ position)
{
	return true;
}