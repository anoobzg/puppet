#include "control_path.h"

ControlPath::ControlPath(unsigned handle, unsigned control_vertex_handle[2], osg::DrawArrays* primitive_set, osg::Array* coord_array): m_handle(handle)
{
	memcpy(m_control_point_handle, control_vertex_handle, 2 * sizeof(unsigned));

	setVertexAttribArray(0, coord_array, osg::Array::BIND_PER_VERTEX);
	addPrimitiveSet(primitive_set);
	setUseVertexBufferObjects(true);
	setUseDisplayList(false);
}

ControlPath::~ControlPath()
{
}

unsigned ControlPath::GetHandle()
{
	return m_handle;
}
