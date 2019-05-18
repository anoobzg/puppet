#include "scene_logic_interface.h"

ControlPoint::ControlPoint()
	:m_vertex_index(-1), m_radius(0.1f)
{
	static unsigned control_point_id = 0;
	m_handle = control_point_id++;

	m_xyz.x = (0.0f);
	m_xyz.y = (0.0f);
	m_xyz.z = (0.0f);
}

ControlPoint::~ControlPoint()
{

}

Path::Path()
{
	static unsigned path_id = 0;
	m_handle = path_id++;

	m_path.clear();
}

Path::~Path()
{

}