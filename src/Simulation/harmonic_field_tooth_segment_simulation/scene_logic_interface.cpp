#include "scene_logic_interface.h"

ControlPoint::ControlPoint()
	:m_vertex_index(-1), m_radius(0.1f)
{
	static unsigned control_point_id = 0;
	m_handle = control_point_id++;
}

ControlPoint::~ControlPoint()
{

}