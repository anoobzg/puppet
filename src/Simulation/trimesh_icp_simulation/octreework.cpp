#include "octreework.h"

OctreeWork::OctreeWork()
	:m_current(0)
{

}

OctreeWork::~OctreeWork()
{

}

void OctreeWork::SetRenderScene(OctreeScene* scene)
{
	m_scene = scene;
}

void OctreeWork::Move(int i)
{
	if (m_meshes.size() != m_points.size())
		m_points.resize(m_meshes.size());

	m_current += i;
	if (m_current < 0) m_current = (int)(m_meshes.size() - 1);
	else if (m_current >= (int)m_meshes.size()) m_current = 0;

	if (!m_points.at(m_current))
		m_points.at(m_current) = new PointsNode(*m_meshes.at(m_current));

	m_scene->RemovePoint(m_current_node);
	m_current_node = m_points.at(m_current);
	m_scene->AddPoint(m_current_node);
}