#include "mappingtask.h"
#include <iostream>
#include <osgWrapper/GeometryCreator.h>

MappingTask::MappingTask(trimesh::CameraData& data, trimesh::TriMesh& target, trimesh::TriMesh& source)
	:m_target(target), m_source(source), m_data(data), m_self(false)
{
}

MappingTask::~MappingTask()
{

}

void MappingTask::SetSelf(bool self)
{
	m_self = self;
}

bool MappingTask::Execute()
{
	m_lines->RemoveAll();

	float fx = m_data.m_fx;
	float fy = m_data.m_fy;
	float cx = m_data.m_cx;
	float cy = m_data.m_cy;
	osg::Vec3Array* coord_array = new osg::Vec3Array();
	coord_array->reserve(1000000);
	size_t target_size = m_target.vertices.size();
	size_t source_size = m_source.vertices.size();
	int width = m_source.grid_width;
	int height = m_source.grid_height;

	if (m_self)
	{
		for (size_t i = 0; i < source_size; i+=3)
		{
			const trimesh::point& p1 = m_source.vertices.at(i);
			// Project source vertex into the destination's image plane.
			int u = (int)roundf((fx * p1[0] + cx * p1[2]) / p1[2]);
			int v = (int)roundf((fy * p1[1] + cy * p1[2]) / p1[2]);

			// Check corresponding vertex
			if ((u >= 0 && u < width - 1) && (v >= 0 && v < height - 1))
			{
				int j = u + v * width;
				if (m_source.grid[j] >= 0 && m_source.grid[j] < m_source.vertices.size())
				{
					trimesh::point p2 = m_source.vertices[m_source.grid[j]];

					coord_array->push_back(osg::Vec3f(p1.x, p1.y, p1.z));
					coord_array->push_back(osg::Vec3f(p2.x, p2.y, p2.z));
				}
			}
		}
	}
	else
	{
		for (size_t i = 0; i < target_size; ++i)
		{
			const trimesh::point& p1 = m_target.vertices.at(i);
			// Project source vertex into the destination's image plane.
			int u = (int)roundf((fx * p1[0] + cx * p1[2]) / p1[2]);
			int v = (int)roundf((fy * p1[1] + cy * p1[2]) / p1[2]);

			// Check corresponding vertex
			if ((u >= 0 && u < width - 1) && (v >= 0 && v < height - 1))
			{
				int j = u + v * width;
				if (m_source.grid[j] >= 0 && m_source.grid[j] < m_source.vertices.size())
				{
					trimesh::point p2 = m_source.vertices[m_source.grid[j]];

					coord_array->push_back(osg::Vec3f(p1.x, p1.y, p1.z));
					coord_array->push_back(osg::Vec3f(p2.x, p2.y, p2.z));
				}
			}
		}
	}
	
	osg::DrawArrays* draw_array = new osg::DrawArrays(GL_LINES, 0, coord_array->size());

	osg::Geometry* geometry = OSGWrapper::GeometryCreator::CreateIndexAttributeGeometry(draw_array, coord_array);
	m_lines->AddChild(geometry);
	return false;
}

void MappingTask::SetAttributeNode(OSGWrapper::AttributeUtilNode* node)
{
	m_lines = node;
}