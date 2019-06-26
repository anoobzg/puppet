#include "octreework.h"
#include "compute_boundingbox.h"
#include <osgWrapper/GeometryCreator.h>

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

void OctreeWork::GenerateChunk()
{
	trimesh::TriMesh* mesh = m_meshes.at(0);
	trimesh::ComputeBoundingbox(mesh->vertices, mesh->bbox);
	m_octree.Initialize(mesh->bbox.center());
	
	//m_octree.Insert(mesh->vertices);

	size_t size = m_octree.m_chunks.size();
	std::vector<OctreeChunk*> chunks;
	for (size_t i = 0; i < size; ++i)
	{
		if (i % 32 == 0) chunks.push_back(&m_octree.m_chunks.at(i));
	}

	float len = m_octree.m_chunk_resolution;
	osg::Vec3Array* coord_array = new osg::Vec3Array();
	osg::DrawElementsUInt* draw_array = new osg::DrawElementsUInt(GL_LINES);

	int offset = 0;
	for (size_t i = 0; i < chunks.size(); ++i)
	{
		OctreeChunk* chunk = chunks.at(i);
		const trimesh::vec3& bmin = chunk->m_bmin;
		osg::Vec3f min = osg::Vec3f(bmin.x, bmin.y, bmin.z);
		osg::Vec3f max = min + osg::Vec3f(len, len, len);

		osg::Vec3f v0 = osg::Vec3f(min.x(), min.y(), min.z());
		osg::Vec3f v1 = osg::Vec3f(max.x(), min.y(), min.z());
		osg::Vec3f v2 = osg::Vec3f(max.x(), min.y(), max.z());
		osg::Vec3f v3 = osg::Vec3f(min.x(), min.y(), max.z());
		osg::Vec3f v4 = osg::Vec3f(min.x(), max.y(), min.z());
		osg::Vec3f v5 = osg::Vec3f(max.x(), max.y(), min.z());
		osg::Vec3f v6 = osg::Vec3f(max.x(), max.y(), max.z());
		osg::Vec3f v7 = osg::Vec3f(min.x(), max.y(), max.z());
		coord_array->push_back(v0);
		coord_array->push_back(v1);
		coord_array->push_back(v2);
		coord_array->push_back(v3);
		coord_array->push_back(v4);
		coord_array->push_back(v5);
		coord_array->push_back(v6);
		coord_array->push_back(v7);

		draw_array->push_back(0 + offset);
		draw_array->push_back(1 + offset);
		draw_array->push_back(1 + offset);
		draw_array->push_back(2 + offset);
		draw_array->push_back(2 + offset);
		draw_array->push_back(3 + offset);
		draw_array->push_back(3 + offset);
		draw_array->push_back(0 + offset);
		draw_array->push_back(4 + offset);
		draw_array->push_back(5 + offset);
		draw_array->push_back(5 + offset);
		draw_array->push_back(6 + offset);
		draw_array->push_back(6 + offset);
		draw_array->push_back(7 + offset);
		draw_array->push_back(7 + offset);
		draw_array->push_back(4 + offset);
		draw_array->push_back(0 + offset);
		draw_array->push_back(4 + offset);
		draw_array->push_back(5 + offset);
		draw_array->push_back(1 + offset);
		draw_array->push_back(2 + offset);
		draw_array->push_back(6 + offset);
		draw_array->push_back(3 + offset);
		draw_array->push_back(7 + offset);

		offset += 8;
	}
	osg::Geometry* geometry = OSGWrapper::GeometryCreator::CreateIndexAttributeGeometry(draw_array, coord_array);
	m_scene->AddOctreeNode(geometry);
}